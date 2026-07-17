#!/usr/bin/env python3
"""Follow GBPlanner's polyline directly with PX4 velocity setpoints.

The controller uses an arc-length lookahead point instead of treating every
path pose as a position setpoint.  Consequently intermediate waypoints are
passed with non-zero velocity; deceleration is applied only near the final
endpoint.  MAVROS converts the ROS ENU velocity/yaw fields to PX4 NED.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


Vec3 = Tuple[float, float, float]

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a: Vec3, value: float) -> Vec3:
    return (a[0] * value, a[1] * value, a[2] * value)


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a: Vec3) -> float:
    return math.sqrt(_dot(a, a))


def _unit(a: Vec3) -> Vec3:
    length = _norm(a)
    return _scale(a, 1.0 / length) if length > 1e-9 else (0.0, 0.0, 0.0)


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass
class Polyline:
    points: List[Vec3]
    cumulative: List[float]

    @property
    def length(self) -> float:
        return self.cumulative[-1]

    def sample(self, distance: float) -> Vec3:
        distance = min(max(distance, 0.0), self.length)
        for index in range(len(self.points) - 1):
            start_s = self.cumulative[index]
            end_s = self.cumulative[index + 1]
            if distance <= end_s or index == len(self.points) - 2:
                segment_length = end_s - start_s
                ratio = 0.0 if segment_length <= 1e-9 else (distance - start_s) / segment_length
                return _add(self.points[index], _scale(_sub(self.points[index + 1], self.points[index]), ratio))
        return self.points[-1]

    def project(self, position: Vec3, minimum_distance: float = 0.0) -> float:
        best_distance_sq = math.inf
        best_s = min(max(minimum_distance, 0.0), self.length)
        for index in range(len(self.points) - 1):
            start_s = self.cumulative[index]
            end_s = self.cumulative[index + 1]
            if end_s < minimum_distance:
                continue
            segment = _sub(self.points[index + 1], self.points[index])
            segment_length_sq = _dot(segment, segment)
            if segment_length_sq <= 1e-12:
                continue
            ratio = min(max(_dot(_sub(position, self.points[index]), segment) / segment_length_sq, 0.0), 1.0)
            candidate = _add(self.points[index], _scale(segment, ratio))
            distance_sq = _dot(_sub(position, candidate), _sub(position, candidate))
            if distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_s = start_s + ratio * (end_s - start_s)
        return best_s


class Px4GbplannerPathFollower(Node):
    def __init__(self) -> None:
        super().__init__('px4_gbplanner_path_follower')

        defaults = {
            'path_topic': '/gbplanner_path',
            'odom_topic': '/mavros/local_position/odom',
            'odom_output_topic': '/rmf/odom',
            'setpoint_topic': '/mavros/setpoint_raw/local',
            'stop_topic': '/planner_control_interface/stop_request',
            'publish_tf': True,
            'tf_parent_frame_id': 'world',
            'tf_child_frame_id': 'base_link',
            'publish_lidar_tf': True,
            'lidar_parent_frame_id': 'base_link',
            'lidar_child_frame_id': 'lidar_link',
            'lidar_z_offset_m': 0.1,
            'control_rate_hz': 30.0,
            'lookahead_distance_m': 1.5,
            'max_speed_mps': 1.5,
            'min_turn_speed_mps': 0.45,
            'max_acceleration_mps2': 0.8,
            'max_deceleration_mps2': 1.2,
            'slowdown_distance_m': 2.5,
            'goal_tolerance_m': 0.30,
            'corner_gain': 1.5,
            'max_yaw_rate_rps': 0.7,
            'yaw_velocity_threshold_mps': 0.20,
            'auto_offboard': True,
            'auto_arm': True,
            'mode_request_period_s': 2.0,
            'prestream_duration_s': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._lookahead = self._positive('lookahead_distance_m')
        self._max_speed = self._positive('max_speed_mps')
        self._min_turn_speed = self._nonnegative('min_turn_speed_mps')
        self._max_accel = self._positive('max_acceleration_mps2')
        self._max_decel = self._positive('max_deceleration_mps2')
        self._slowdown_distance = self._positive('slowdown_distance_m')
        self._goal_tolerance = self._nonnegative('goal_tolerance_m')
        self._corner_gain = self._nonnegative('corner_gain')
        self._max_yaw_rate = self._positive('max_yaw_rate_rps')
        self._yaw_velocity_threshold = self._nonnegative('yaw_velocity_threshold_mps')
        self._auto_offboard = bool(self.get_parameter('auto_offboard').value)
        self._auto_arm = bool(self.get_parameter('auto_arm').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._tf_parent_frame = str(self.get_parameter('tf_parent_frame_id').value)
        self._tf_child_frame = str(self.get_parameter('tf_child_frame_id').value)
        self._publish_lidar_tf = bool(self.get_parameter('publish_lidar_tf').value)
        self._lidar_parent_frame = str(self.get_parameter('lidar_parent_frame_id').value)
        self._lidar_child_frame = str(self.get_parameter('lidar_child_frame_id').value)
        self._lidar_z_offset = float(self.get_parameter('lidar_z_offset_m').value)
        self._request_period = self._positive('mode_request_period_s')
        self._prestream_duration = self._nonnegative('prestream_duration_s')
        rate_hz = self._positive('control_rate_hz')
        if rate_hz <= 2.0:
            raise ValueError('control_rate_hz must exceed PX4\'s 2 Hz offboard minimum')
        self._period = 1.0 / rate_hz

        path_topic = str(self.get_parameter('path_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        odom_output_topic = str(self.get_parameter('odom_output_topic').value)
        setpoint_topic = str(self.get_parameter('setpoint_topic').value)
        stop_topic = str(self.get_parameter('stop_topic').value)

        self._setpoint_pub = self.create_publisher(PositionTarget, setpoint_topic, 10)
        self._odom_pub = self.create_publisher(Odometry, odom_output_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        self.create_subscription(Path, path_topic, self._path_callback, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_callback, _SENSOR_QOS)
        self.create_subscription(State, '/mavros/state', self._state_callback, _SENSOR_QOS)
        self.create_subscription(Bool, stop_topic, self._stop_callback, 10)

        self._set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self._arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        self._path: Optional[Polyline] = None
        self._position: Optional[Vec3] = None
        self._progress = 0.0
        self._command_velocity: Vec3 = (0.0, 0.0, 0.0)
        self._command_yaw = 0.0
        self._state = State()
        self._stopped = False
        self._last_tick_ns: Optional[int] = None
        self._prestream_start_ns: Optional[int] = None
        self._last_request_ns = 0
        self._mode_request_pending = False
        self._arm_request_pending = False
        self.create_timer(self._period, self._control_tick)

        self.get_logger().info(
            f'Following {path_topic} directly on {setpoint_topic}: '
            f'lookahead={self._lookahead:.2f} m, max_speed={self._max_speed:.2f} m/s'
        )

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f'{name} must be positive')
        return value

    def _nonnegative(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value < 0.0:
            raise ValueError(f'{name} must be non-negative')
        return value

    def _path_callback(self, message: Path) -> None:
        points: List[Vec3] = []
        for pose in message.poses:
            point = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            if not points or _norm(_sub(point, points[-1])) > 1e-3:
                points.append(point)
        if len(points) < 2:
            self.get_logger().warning('Ignoring GBPlanner path with fewer than two distinct points')
            return

        cumulative = [0.0]
        for start, end in zip(points, points[1:]):
            cumulative.append(cumulative[-1] + _norm(_sub(end, start)))
        self._path = Polyline(points, cumulative)
        self._progress = self._path.project(self._position) if self._position is not None else 0.0
        self._stopped = False
        self.get_logger().info(
            f'Accepted GBPlanner path: {len(points)} points, {self._path.length:.1f} m'
        )

    def _odom_callback(self, message: Odometry) -> None:
        self._odom_pub.publish(message)
        self._publish_odom_tf(message)
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self._position = (position.x, position.y, position.z)
        if self._last_tick_ns is None:
            self._command_yaw = _yaw_from_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w
            )

    def _publish_odom_tf(self, odom: Odometry) -> None:
        if self._tf_broadcaster is None:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._tf_parent_frame or odom.header.frame_id
        transform.child_frame_id = self._tf_child_frame or odom.child_frame_id
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

        if self._publish_lidar_tf:
            lidar_transform = TransformStamped()
            lidar_transform.header.stamp = transform.header.stamp
            lidar_transform.header.frame_id = self._lidar_parent_frame
            lidar_transform.child_frame_id = self._lidar_child_frame
            lidar_transform.transform.translation.z = self._lidar_z_offset
            lidar_transform.transform.rotation.w = 1.0
            self._tf_broadcaster.sendTransform(lidar_transform)

    def _state_callback(self, message: State) -> None:
        self._state = message

    def _stop_callback(self, _message: Bool) -> None:
        # The PCI publishes a default-constructed Bool, so reception itself is
        # the stop event; its data field is not meaningful.
        self._stopped = True
        self._path = None
        self.get_logger().warning('Planner stop received; commanding zero velocity')

    def _control_tick(self) -> None:
        if self._position is None:
            return
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        dt = self._period if self._last_tick_ns is None else min(max((now_ns - self._last_tick_ns) * 1e-9, 1e-3), 0.2)
        self._last_tick_ns = now_ns

        desired = self._desired_velocity()
        rate_limit = self._max_decel if _norm(desired) < _norm(self._command_velocity) else self._max_accel
        self._command_velocity = self._slew_vector(self._command_velocity, desired, rate_limit * dt)
        self._update_yaw(dt)
        self._publish_setpoint(now)

        if self._prestream_start_ns is None:
            self._prestream_start_ns = now_ns
        if self._path is not None and not self._stopped:
            self._request_offboard_and_arm(now_ns)

    def _desired_velocity(self) -> Vec3:
        if self._path is None or self._stopped or self._position is None:
            return (0.0, 0.0, 0.0)

        projected = self._path.project(self._position, max(0.0, self._progress - 0.25))
        self._progress = max(self._progress, projected)
        remaining = max(0.0, self._path.length - self._progress)
        distance_to_goal = _norm(_sub(self._path.points[-1], self._position))
        if remaining <= self._goal_tolerance and distance_to_goal <= self._goal_tolerance:
            return (0.0, 0.0, 0.0)

        carrot_s = min(self._progress + self._lookahead, self._path.length)
        carrot = self._path.sample(carrot_s)
        direction = _unit(_sub(carrot, self._position))
        if _norm(direction) <= 1e-9:
            direction = _unit(_sub(self._path.points[-1], self._position))

        speed = self._max_speed
        braking_distance = max(remaining, distance_to_goal)
        if braking_distance < self._slowdown_distance:
            # Kinematic braking envelope.  Only the path endpoint can request
            # zero speed; intermediate vertices are deliberately ignored.
            braking_speed = math.sqrt(
                2.0 * self._max_decel * max(braking_distance - self._goal_tolerance, 0.0)
            )
            speed = min(speed, braking_speed)

        tangent_near = _unit(_sub(
            self._path.sample(min(carrot_s + 0.25, self._path.length)),
            self._path.sample(max(carrot_s - 0.25, 0.0)),
        ))
        tangent_far = _unit(_sub(
            self._path.sample(min(carrot_s + self._lookahead, self._path.length)),
            carrot,
        ))
        if _norm(tangent_near) > 0.0 and _norm(tangent_far) > 0.0:
            turn_angle = math.acos(min(max(_dot(tangent_near, tangent_far), -1.0), 1.0))
            corner_speed = max(self._min_turn_speed, self._max_speed / (1.0 + self._corner_gain * turn_angle))
            speed = min(speed, corner_speed)

        return _scale(direction, speed)

    @staticmethod
    def _slew_vector(current: Vec3, target: Vec3, max_delta: float) -> Vec3:
        delta = _sub(target, current)
        delta_norm = _norm(delta)
        if delta_norm <= max_delta or delta_norm <= 1e-9:
            return target
        return _add(current, _scale(delta, max_delta / delta_norm))

    def _update_yaw(self, dt: float) -> None:
        horizontal_speed = math.hypot(self._command_velocity[0], self._command_velocity[1])
        if horizontal_speed < self._yaw_velocity_threshold:
            return
        target_yaw = math.atan2(self._command_velocity[1], self._command_velocity[0])
        yaw_delta = _wrap_pi(target_yaw - self._command_yaw)
        max_delta = self._max_yaw_rate * dt
        self._command_yaw = _wrap_pi(self._command_yaw + min(max(yaw_delta, -max_delta), max_delta))

    def _publish_setpoint(self, stamp) -> None:
        target = PositionTarget()
        target.header.stamp = stamp.to_msg()
        target.header.frame_id = 'map'
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        target.velocity.x, target.velocity.y, target.velocity.z = self._command_velocity
        target.yaw = self._command_yaw
        self._setpoint_pub.publish(target)

    def _request_offboard_and_arm(self, now_ns: int) -> None:
        if not self._state.connected or self._prestream_start_ns is None:
            return
        if (now_ns - self._prestream_start_ns) * 1e-9 < self._prestream_duration:
            return
        if (now_ns - self._last_request_ns) * 1e-9 < self._request_period:
            return

        requested = False
        if self._auto_offboard and self._state.mode != 'OFFBOARD' and not self._mode_request_pending and self._set_mode_client.service_is_ready():
            request = SetMode.Request()
            request.custom_mode = 'OFFBOARD'
            future = self._set_mode_client.call_async(request)
            future.add_done_callback(self._mode_request_done)
            self._mode_request_pending = True
            requested = True
        if self._auto_arm and not self._state.armed and not self._arm_request_pending and self._arming_client.service_is_ready():
            request = CommandBool.Request()
            request.value = True
            future = self._arming_client.call_async(request)
            future.add_done_callback(self._arm_request_done)
            self._arm_request_pending = True
            requested = True
        if requested:
            self._last_request_ns = now_ns

    def _mode_request_done(self, future) -> None:
        self._mode_request_pending = False
        try:
            if not future.result().mode_sent:
                self.get_logger().warning('PX4 rejected OFFBOARD mode request')
        except Exception as exception:  # noqa: BLE001
            self.get_logger().warning(f'OFFBOARD mode request failed: {exception}')

    def _arm_request_done(self, future) -> None:
        self._arm_request_pending = False
        try:
            if not future.result().success:
                self.get_logger().warning('PX4 rejected arming request')
        except Exception as exception:  # noqa: BLE001
            self.get_logger().warning(f'Arming request failed: {exception}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4GbplannerPathFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
