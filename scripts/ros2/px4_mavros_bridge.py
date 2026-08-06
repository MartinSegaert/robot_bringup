#!/usr/bin/env python3
"""Connect the stack's acceleration/velocity and odometry topics to MAVROS.

Commands are selected by ``/use_gbplanner`` and kept alive at
``setpoint_rate_hz`` while fresh. GBPlanner uses acceleration control; the
fallback waypoint controller uses velocity control.
Publishing stops after ``setpoint_timeout_s`` so PX4 can detect loss of the
offboard command stream instead of flying indefinitely on a stale command.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

_SELECTOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _quaternion_from_rpy(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    """Return an xyzw quaternion for fixed-axis roll, pitch, and yaw."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _load_fixed_joint_pose(
    sdf_path: str, parent_link: str, child_link: str
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Load parent-to-child xyz and xyzw from a fixed joint in an SDF file."""
    path = Path(sdf_path).expanduser()
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as error:
        raise ValueError(f'Cannot read LiDAR model SDF {path}: {error}') from error

    matching_joints = []
    for joint in root.iter('joint'):
        parent = (joint.findtext('parent') or '').strip()
        child = (joint.findtext('child') or '').strip()
        if parent == parent_link and child == child_link:
            matching_joints.append(joint)

    if not matching_joints:
        raise ValueError(
            f'No joint from {parent_link!r} to {child_link!r} found in {path}'
        )
    if len(matching_joints) > 1:
        raise ValueError(
            f'Multiple joints from {parent_link!r} to {child_link!r} found in {path}'
        )

    joint = matching_joints[0]
    if joint.get('type') != 'fixed':
        raise ValueError(
            f'LiDAR joint {joint.get("name", "<unnamed>")!r} in {path} '
            'must be fixed'
        )

    pose = joint.find('pose')
    if pose is None or not (pose.text or '').strip():
        values = [0.0] * 6
        rotation_format = 'euler_rpy'
    else:
        relative_to = (pose.get('relative_to') or parent_link).strip()
        if relative_to != parent_link:
            raise ValueError(
                f'LiDAR joint pose in {path} is relative to {relative_to!r}; '
                f'expected {parent_link!r}'
            )
        rotation_format = pose.get('rotation_format', 'euler_rpy')
        try:
            values = [float(value) for value in pose.text.split()]
        except ValueError as error:
            raise ValueError(f'Invalid LiDAR joint pose in {path}') from error

    translation = tuple(values[:3])
    if rotation_format == 'euler_rpy':
        if len(values) != 6:
            raise ValueError(
                f'LiDAR joint pose in {path} must contain 6 values for euler_rpy'
            )
        roll, pitch, yaw = values[3:]
        if pose is not None and pose.get('degrees', 'false').lower() == 'true':
            roll, pitch, yaw = map(math.radians, (roll, pitch, yaw))
        rotation = _quaternion_from_rpy(roll, pitch, yaw)
    elif rotation_format == 'quat_xyzw':
        if len(values) != 7:
            raise ValueError(
                f'LiDAR joint pose in {path} must contain 7 values for quat_xyzw'
            )
        rotation = tuple(values[3:])
        norm = math.sqrt(sum(component * component for component in rotation))
        if norm == 0.0:
            raise ValueError(f'LiDAR joint quaternion in {path} has zero norm')
        rotation = tuple(component / norm for component in rotation)
    else:
        raise ValueError(
            f'Unsupported LiDAR pose rotation_format {rotation_format!r} in {path}'
        )

    return translation, rotation


class Px4MavrosBridge(Node):
    def __init__(self) -> None:
        super().__init__('px4_mavros_bridge')

        self.declare_parameter('accel_input_topic', '/rmf/cmd/acc')
        self.declare_parameter('velocity_input_topic', '/rmf/cmd/vel')
        self.declare_parameter('use_gbplanner_topic', '/use_gbplanner')
        self.declare_parameter('controller_ready_topic', '')
        self.declare_parameter('controller_hold_service', '')
        self.declare_parameter('setpoint_output_topic', '/mavros/setpoint_raw/local')
        self.declare_parameter('mavros_odom_topic', '/mavros/local_position/odom')
        self.declare_parameter('odom_output_topic', '/rmf/odom')
        self.declare_parameter('body_frame_id', 'rmf/base_link')
        self.declare_parameter('tf_parent_frame_id', '')
        self.declare_parameter('tf_child_frame_id', '')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('stamp_tf_with_ros_time', True)
        self.declare_parameter('publish_lidar_tf', True)
        self.declare_parameter('lidar_parent_frame_id', 'base_link')
        self.declare_parameter('lidar_child_frame_id', 'lidar_link')
        self.declare_parameter('lidar_model_sdf_path', '')
        self.declare_parameter('lidar_z_offset_m', 0.1)
        self.declare_parameter('setpoint_rate_hz', 50.0)
        self.declare_parameter('setpoint_timeout_s', 0.5)
        self.declare_parameter('auto_offboard', False)
        self.declare_parameter('auto_arm', False)
        self.declare_parameter('auto_takeoff', False)
        self.declare_parameter('takeoff_handover_altitude', 2.0)
        self.declare_parameter('takeoff_altitude_tolerance', 0.2)
        self.declare_parameter('takeoff_hold_mode', 'AUTO.LOITER')
        self.declare_parameter('mode_request_period_s', 2.0)

        accel_topic = self.get_parameter('accel_input_topic').value
        velocity_topic = self.get_parameter('velocity_input_topic').value
        use_gbplanner_topic = self.get_parameter('use_gbplanner_topic').value
        controller_ready_topic = str(
            self.get_parameter('controller_ready_topic').value
        ).strip()
        self._controller_hold_service = str(
            self.get_parameter('controller_hold_service').value
        ).strip()
        setpoint_topic = self.get_parameter('setpoint_output_topic').value
        mavros_odom_topic = self.get_parameter('mavros_odom_topic').value
        odom_topic = self.get_parameter('odom_output_topic').value
        self._body_frame_id = self.get_parameter('body_frame_id').value
        self._tf_parent_frame_id = self.get_parameter('tf_parent_frame_id').value
        self._tf_child_frame_id = self.get_parameter('tf_child_frame_id').value
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._stamp_tf_with_ros_time = bool(self.get_parameter('stamp_tf_with_ros_time').value)
        self._publish_lidar_tf = bool(self.get_parameter('publish_lidar_tf').value)
        self._lidar_parent_frame_id = self.get_parameter('lidar_parent_frame_id').value
        self._lidar_child_frame_id = self.get_parameter('lidar_child_frame_id').value
        lidar_model_sdf_path = self.get_parameter('lidar_model_sdf_path').value
        self._lidar_z_offset_m = float(self.get_parameter('lidar_z_offset_m').value)
        self._lidar_translation = (0.0, 0.0, self._lidar_z_offset_m)
        self._lidar_rotation = (0.0, 0.0, 0.0, 1.0)
        self._lidar_pose_source = 'lidar_z_offset_m fallback'
        if lidar_model_sdf_path:
            self._lidar_translation, self._lidar_rotation = _load_fixed_joint_pose(
                lidar_model_sdf_path,
                self._lidar_parent_frame_id,
                self._lidar_child_frame_id,
            )
            self._lidar_pose_source = lidar_model_sdf_path
        rate_hz = float(self.get_parameter('setpoint_rate_hz').value)
        self._timeout_s = float(self.get_parameter('setpoint_timeout_s').value)
        self._auto_offboard = bool(self.get_parameter('auto_offboard').value)
        self._auto_arm = bool(self.get_parameter('auto_arm').value)
        self._auto_takeoff = bool(self.get_parameter('auto_takeoff').value)
        self._takeoff_handover_altitude = float(
            self.get_parameter('takeoff_handover_altitude').value
        )
        self._takeoff_altitude_tolerance = float(
            self.get_parameter('takeoff_altitude_tolerance').value
        )
        self._takeoff_hold_mode = str(
            self.get_parameter('takeoff_hold_mode').value
        ).strip()
        self._mode_request_period_s = float(self.get_parameter('mode_request_period_s').value)

        if rate_hz <= 2.0:
            raise ValueError('setpoint_rate_hz must be greater than PX4\'s 2 Hz offboard minimum')
        if self._timeout_s <= 0.0:
            raise ValueError('setpoint_timeout_s must be positive')
        if self._takeoff_handover_altitude <= 0.0:
            raise ValueError('takeoff_handover_altitude must be positive')
        if self._takeoff_altitude_tolerance < 0.0:
            raise ValueError('takeoff_altitude_tolerance must be non-negative')
        if not self._takeoff_hold_mode:
            raise ValueError('takeoff_hold_mode must not be empty')
        if self._mode_request_period_s <= 0.0:
            raise ValueError('mode_request_period_s must be positive')

        self._setpoint_pub = self.create_publisher(PositionTarget, setpoint_topic, 10)
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        self._use_gbplanner = True
        self.create_subscription(Twist, accel_topic, self._accel_callback, 10)
        self.create_subscription(Twist, velocity_topic, self._velocity_callback, 10)
        self.create_subscription(
            Bool,
            use_gbplanner_topic,
            self._use_gbplanner_callback,
            _SELECTOR_QOS,
        )
        self._separate_controller_ready_topic = bool(
            controller_ready_topic and controller_ready_topic != accel_topic
        )
        if self._separate_controller_ready_topic:
            self.create_subscription(
                Twist,
                controller_ready_topic,
                self._controller_ready_callback,
                10,
            )
        self.create_subscription(Odometry, mavros_odom_topic, self._odom_callback, _SENSOR_QOS)
        self.create_subscription(State, '/mavros/state', self._state_callback, _SENSOR_QOS)

        self._set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self._arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._controller_hold_client = (
            self.create_client(Trigger, self._controller_hold_service)
            if self._controller_hold_service
            else None
        )

        self._last_setpoint: Optional[PositionTarget] = None
        self._last_setpoint_ns: Optional[int] = None
        self._last_controller_command_ns: Optional[int] = None
        self._state = State()
        self._has_odom = False
        self._latest_odom_z = 0.0
        self._takeoff_origin_z: Optional[float] = None
        self._takeoff_complete = not self._auto_takeoff
        self._last_mode_request_ns = 0
        self._mode_request_pending = False
        self._pending_mode_name: Optional[str] = None
        self._arm_request_pending = False
        self._controller_hold_ready = not bool(self._controller_hold_service)
        self._controller_hold_request_pending = False
        self._last_controller_hold_request_ns = 0
        self._timed_out = False
        self.create_timer(1.0 / rate_hz, self._publish_setpoint)

        self.get_logger().info(
            f'Acceleration {accel_topic} when {use_gbplanner_topic}=true; '
            f'velocity {velocity_topic} when {use_gbplanner_topic}=false; '
            f'setpoints -> {setpoint_topic}; '
            f'odometry {mavros_odom_topic} -> {odom_topic}'
        )
        if self._separate_controller_ready_topic:
            self.get_logger().info(
                f'OFFBOARD readiness requires fresh upstream commands on '
                f'{controller_ready_topic}'
            )
        if self._controller_hold_service:
            self.get_logger().info(
                f'OFFBOARD handover will first latch the airborne reference via '
                f'{self._controller_hold_service}'
            )
        self.get_logger().info(
            f'LiDAR TF {self._lidar_parent_frame_id} -> '
            f'{self._lidar_child_frame_id}: xyz={self._lidar_translation}, '
            f'xyzw={self._lidar_rotation} (source: {self._lidar_pose_source})'
        )
        if self._auto_takeoff:
            self.get_logger().info(
                'Automatic startup sequence enabled: arm in AUTO.TAKEOFF, '
                f'then hold in {self._takeoff_hold_mode} near '
                f'{self._takeoff_handover_altitude:.2f} m AGL '
                f'(tolerance {self._takeoff_altitude_tolerance:.2f} m) until '
                'a fresh controller command allows OFFBOARD'
            )

    def _accel_callback(self, command: Twist) -> None:
        if not self._use_gbplanner:
            return

        target = PositionTarget()
        target.header.frame_id = self._body_frame_id
        target.coordinate_frame = PositionTarget.FRAME_BODY_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_YAW
        )
        target.acceleration_or_force.x = command.linear.x
        target.acceleration_or_force.y = command.linear.y
        target.acceleration_or_force.z = command.linear.z
        target.yaw_rate = command.angular.z

        self._last_setpoint = target
        self._last_setpoint_ns = self.get_clock().now().nanoseconds
        if not self._separate_controller_ready_topic:
            self._last_controller_command_ns = self._last_setpoint_ns

    def _velocity_callback(self, command: Twist) -> None:
        if self._use_gbplanner:
            return

        target = PositionTarget()
        target.header.frame_id = self._body_frame_id
        target.coordinate_frame = PositionTarget.FRAME_BODY_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
        )
        target.velocity.x = command.linear.x
        target.velocity.y = command.linear.y
        target.velocity.z = command.linear.z
        target.yaw_rate = command.angular.z

        self._last_setpoint = target
        self._last_setpoint_ns = self.get_clock().now().nanoseconds
        # The optional readiness topic belongs to the acceleration controller.
        # A velocity message is itself proof that the fallback controller is
        # ready, so it must refresh both timestamps.
        self._last_controller_command_ns = self._last_setpoint_ns

    def _use_gbplanner_callback(self, selection: Bool) -> None:
        use_gbplanner = bool(selection.data)
        if use_gbplanner == self._use_gbplanner:
            return

        self._use_gbplanner = use_gbplanner
        # Never replay a cached command encoded for the previously active
        # control mode. Wait for a fresh command from the selected controller.
        self._last_setpoint = None
        self._last_setpoint_ns = None
        self._last_controller_command_ns = None
        mode = 'acceleration' if use_gbplanner else 'velocity'
        self.get_logger().info(f'Switched to {mode} control')

    def _controller_ready_callback(self, _command: Twist) -> None:
        """Record a real upstream command independently of CBF fallback output."""
        self._last_controller_command_ns = self.get_clock().now().nanoseconds

    def _publish_setpoint(self) -> None:
        now = self.get_clock().now()
        command_is_fresh = False
        setpoint_age_s = math.inf
        controller_age_s = math.inf
        if self._last_setpoint is not None and self._last_setpoint_ns is not None:
            setpoint_age_s = (now.nanoseconds - self._last_setpoint_ns) * 1e-9
        if self._last_controller_command_ns is not None:
            controller_age_s = (
                now.nanoseconds - self._last_controller_command_ns
            ) * 1e-9
        command_is_fresh = (
            setpoint_age_s <= self._timeout_s
            and controller_age_s <= self._timeout_s
        )

        # AUTO.TAKEOFF and arming do not depend on the NMPC command stream.
        # OFFBOARD handover does: PX4 must see fresh setpoints before accepting
        # the mode, and must never enter it when the controller is unavailable.
        self._request_offboard_and_arm(now.nanoseconds, command_is_fresh)

        if not command_is_fresh:
            if not self._timed_out:
                finite_ages = [
                    age
                    for age in (setpoint_age_s, controller_age_s)
                    if math.isfinite(age)
                ]
                if finite_ages:
                    self.get_logger().warning(
                        'Controller/setpoint stream timed out after '
                        f'{max(finite_ages):.2f} s; '
                        'stopping setpoints'
                    )
                self._timed_out = True
            return

        self._timed_out = False
        self._last_setpoint.header.stamp = now.to_msg()
        self._setpoint_pub.publish(self._last_setpoint)

    def _odom_callback(self, odom: Odometry) -> None:
        self._has_odom = True
        self._latest_odom_z = float(odom.pose.pose.position.z)
        if self._takeoff_origin_z is None:
            self._takeoff_origin_z = self._latest_odom_z
            self.get_logger().info(
                f'Takeoff altitude origin set to z={self._takeoff_origin_z:.2f} m'
            )
        self._odom_pub.publish(odom)
        self._publish_odom_tf(odom)

    def _publish_odom_tf(self, odom: Odometry) -> None:
        if self._tf_broadcaster is None:
            return

        parent_frame = self._tf_parent_frame_id or odom.header.frame_id
        child_frame = self._tf_child_frame_id or odom.child_frame_id
        if not parent_frame or not child_frame:
            self.get_logger().warning(
                'Cannot publish odometry TF without parent and child frame ids',
                throttle_duration_sec=5.0,
            )
            return

        transform = TransformStamped()
        transform.header.stamp = (
            self.get_clock().now().to_msg()
            if self._stamp_tf_with_ros_time
            else odom.header.stamp
        )
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

        if self._publish_lidar_tf:
            lidar_transform = TransformStamped()
            lidar_transform.header.stamp = transform.header.stamp
            lidar_transform.header.frame_id = self._lidar_parent_frame_id
            lidar_transform.child_frame_id = self._lidar_child_frame_id
            (
                lidar_transform.transform.translation.x,
                lidar_transform.transform.translation.y,
                lidar_transform.transform.translation.z,
            ) = self._lidar_translation
            (
                lidar_transform.transform.rotation.x,
                lidar_transform.transform.rotation.y,
                lidar_transform.transform.rotation.z,
                lidar_transform.transform.rotation.w,
            ) = self._lidar_rotation
            self._tf_broadcaster.sendTransform(lidar_transform)

    def _state_callback(self, state: State) -> None:
        self._state = state

    def _request_offboard_and_arm(
        self, now_ns: int, offboard_command_ready: bool
    ) -> None:
        if not self._has_odom or not self._state.connected:
            return

        if not offboard_command_ready and self._controller_hold_service:
            # A controller restart must latch a new hold pose before it can
            # regain OFFBOARD control.
            self._controller_hold_ready = False

        desired_mode: Optional[str] = None
        if self._auto_takeoff and not self._takeoff_complete:
            altitude_agl = self._altitude_above_takeoff_origin()
            handover_altitude = max(
                0.0,
                self._takeoff_handover_altitude - self._takeoff_altitude_tolerance,
            )
            if self._state.armed and altitude_agl >= handover_altitude:
                self._takeoff_complete = True
                # A recent AUTO.TAKEOFF request must not delay the safety
                # transition into the holding mode.
                self._last_mode_request_ns = 0
                self.get_logger().info(
                    f'Takeoff reached {altitude_agl:.2f} m AGL; '
                    f'entering {self._takeoff_hold_mode} until a fresh '
                    'controller command is available'
                )
            else:
                desired_mode = 'AUTO.TAKEOFF'

        if (
            self._takeoff_complete
            and offboard_command_ready
            and not self._controller_hold_ready
        ):
            self._request_controller_hold(now_ns)

        offboard_handover_ready = (
            offboard_command_ready and self._controller_hold_ready
        )

        # Do not leave PX4 in AUTO.TAKEOFF after reaching the handover
        # altitude. Without a fresh controller command it would continue to
        # PX4's own, potentially higher, takeoff target. Holding explicitly
        # also gives controllers that start later a stable state from which to
        # enter OFFBOARD.
        if (
            self._takeoff_complete
            and self._auto_takeoff
            and not (self._auto_offboard and offboard_handover_ready)
        ):
            desired_mode = self._takeoff_hold_mode

        if (
            self._takeoff_complete
            and self._auto_offboard
            and offboard_handover_ready
        ):
            desired_mode = 'OFFBOARD'

        elapsed_s = (now_ns - self._last_mode_request_ns) * 1e-9
        if elapsed_s < self._mode_request_period_s:
            return

        requested = False
        if (
            desired_mode is not None
            and self._state.mode != desired_mode
            and not self._mode_request_pending
            and self._set_mode_client.service_is_ready()
        ):
            request = SetMode.Request()
            request.custom_mode = desired_mode
            future = self._set_mode_client.call_async(request)
            future.add_done_callback(self._mode_request_done)
            self._mode_request_pending = True
            self._pending_mode_name = desired_mode
            requested = True

        arming_mode_ready = (
            not self._auto_takeoff
            or self._takeoff_complete
            or self._state.mode == 'AUTO.TAKEOFF'
        )
        if (
            self._auto_arm
            and arming_mode_ready
            and not self._state.armed
            and not self._arm_request_pending
            and self._arming_client.service_is_ready()
        ):
            request = CommandBool.Request()
            request.value = True
            future = self._arming_client.call_async(request)
            future.add_done_callback(self._arm_request_done)
            self._arm_request_pending = True
            requested = True

        if requested:
            self._last_mode_request_ns = now_ns

    def _request_controller_hold(self, now_ns: int) -> None:
        if (
            self._controller_hold_client is None
            or self._controller_hold_request_pending
            or not self._controller_hold_client.service_is_ready()
        ):
            return

        elapsed_s = (
            now_ns - self._last_controller_hold_request_ns
        ) * 1e-9
        if elapsed_s < self._mode_request_period_s:
            return

        future = self._controller_hold_client.call_async(Trigger.Request())
        future.add_done_callback(self._controller_hold_request_done)
        self._controller_hold_request_pending = True
        self._last_controller_hold_request_ns = now_ns

    def _controller_hold_request_done(self, future) -> None:
        self._controller_hold_request_pending = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - ROS client transport errors.
            self.get_logger().warning(
                f'Controller hover-reference request failed: {exc}'
            )
            return
        if not response.success:
            self.get_logger().warning('Controller rejected hover-reference request')
            return
        self._controller_hold_ready = True
        self.get_logger().info(
            'Airborne hover reference latched; OFFBOARD handover enabled'
        )

    def _altitude_above_takeoff_origin(self) -> float:
        if self._takeoff_origin_z is None:
            return 0.0
        return max(0.0, self._latest_odom_z - self._takeoff_origin_z)

    def _mode_request_done(self, future) -> None:
        mode_name = self._pending_mode_name or 'flight'
        self._mode_request_pending = False
        self._pending_mode_name = None
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - ROS client exceptions include transport details.
            self.get_logger().warning(f'{mode_name} mode request failed: {exc}')
            return
        if not response.mode_sent:
            self.get_logger().warning(f'{mode_name} mode request was rejected')

    def _arm_request_done(self, future) -> None:
        self._arm_request_pending = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - ROS client exceptions include transport details.
            self.get_logger().warning(f'Arming request failed: {exc}')
            return
        if not response.success:
            self.get_logger().warning('Arming request was rejected')


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[Px4MavrosBridge] = None
    try:
        node = Px4MavrosBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # Humble's signal handler can invalidate the context while the
        # executor is rebuilding its wait set. That surfaces as RCLError
        # instead of KeyboardInterrupt / ExternalShutdownException. Suppress
        # only that shutdown race; real runtime errors still propagate.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
