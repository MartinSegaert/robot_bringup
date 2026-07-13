#!/usr/bin/env python3
"""Convert RViz 2D goal poses into NMPC joystick velocity commands."""

import math
from typing import Optional, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Transform, Twist, Vector3
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _rotate_world_to_body(
    vec: Tuple[float, float, float],
    quat_xyzw: Tuple[float, float, float, float],
) -> Tuple[float, float, float]:
    """Rotate a world-frame vector into the body frame using q_world_body."""
    x, y, z, w = quat_xyzw
    vx, vy, vz = vec

    # R(q)^T * v, expanded to avoid adding a tf dependency for one operation.
    r00 = 1.0 - 2.0 * (y * y + z * z)
    r01 = 2.0 * (x * y - z * w)
    r02 = 2.0 * (x * z + y * w)
    r10 = 2.0 * (x * y + z * w)
    r11 = 1.0 - 2.0 * (x * x + z * z)
    r12 = 2.0 * (y * z - x * w)
    r20 = 2.0 * (x * z - y * w)
    r21 = 2.0 * (y * z + x * w)
    r22 = 1.0 - 2.0 * (x * x + y * y)

    return (
        r00 * vx + r10 * vy + r20 * vz,
        r01 * vx + r11 * vy + r21 * vz,
        r02 * vx + r12 * vy + r22 * vz,
    )


def _yaw_from_quat(quat_xyzw: Tuple[float, float, float, float]) -> float:
    x, y, z, w = quat_xyzw
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _quat_from_yaw(yaw: float) -> Quaternion:
    half_yaw = 0.5 * yaw
    return Quaternion(w=math.cos(half_yaw), x=0.0, y=0.0, z=math.sin(half_yaw))


class GoalVelocityCommander(Node):
    def __init__(self) -> None:
        super().__init__('goal_velocity_commander')

        self.declare_parameter('goal_topic', '/goal')
        self.declare_parameter('secondary_goal_topic', '/goal_pose')
        self.declare_parameter('odometry_topic', '/rmf/odom')
        self.declare_parameter('cmd_topic', '/goal_cmd_vel')
        self.declare_parameter('horizon_ref_topic', '/sdf_nmpc/horizon_ref')
        self.declare_parameter('horizon_viz_topic', '/sdf_nmpc/viz/horizon_ref')
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('target_altitude', 2.0)
        self.declare_parameter('goal_tolerance', 0.4)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('body_frame_output', True)
        self.declare_parameter('publish_horizon_ref', True)
        self.declare_parameter('horizon_steps', 20)
        self.declare_parameter('horizon_time', 1.5)
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('align_yaw_to_velocity', True)

        goal_topic = self.get_parameter('goal_topic').value
        secondary_goal_topic = self.get_parameter('secondary_goal_topic').value
        odometry_topic = self.get_parameter('odometry_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value
        horizon_ref_topic = self.get_parameter('horizon_ref_topic').value
        horizon_viz_topic = self.get_parameter('horizon_viz_topic').value
        publish_rate = max(float(self.get_parameter('publish_rate').value), 1.0)

        self._speed = max(float(self.get_parameter('speed').value), 0.0)
        self._target_altitude = float(self.get_parameter('target_altitude').value)
        self._goal_tolerance = max(float(self.get_parameter('goal_tolerance').value), 0.0)
        self._body_frame_output = bool(self.get_parameter('body_frame_output').value)
        self._publish_horizon_ref = bool(self.get_parameter('publish_horizon_ref').value)
        self._horizon_steps = max(int(self.get_parameter('horizon_steps').value), 1)
        self._horizon_time = max(float(self.get_parameter('horizon_time').value), 0.01)
        self._world_frame = str(self.get_parameter('world_frame').value)
        self._align_yaw_to_velocity = bool(self.get_parameter('align_yaw_to_velocity').value)

        self._goal: Optional[Tuple[float, float, float]] = None
        self._odom: Optional[Odometry] = None

        self._pub = self.create_publisher(Twist, cmd_topic, 10)
        self._horizon_pub = self.create_publisher(MultiDOFJointTrajectory, horizon_ref_topic, 1)
        self._horizon_viz_pub = self.create_publisher(Path, horizon_viz_topic, 1)
        self.create_subscription(PoseStamped, goal_topic, self._goal_cb, 10)
        if secondary_goal_topic and secondary_goal_topic != goal_topic:
            self.create_subscription(PoseStamped, secondary_goal_topic, self._goal_cb, 10)
        self.create_subscription(Odometry, odometry_topic, self._odom_cb, _SENSOR_QOS)
        self.create_timer(1.0 / publish_rate, self._timer_cb)

        frame_label = 'body' if self._body_frame_output else 'world'
        self.get_logger().info(
            f'Publishing {frame_label}-frame velocity commands on {cmd_topic}; '
            f'listening for goals on {goal_topic}, odometry on {odometry_topic}'
        )

    def _goal_cb(self, msg: PoseStamped) -> None:
        self._goal = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            self._target_altitude,
        )
        self.get_logger().info(
            'New goal: '
            f'x={self._goal[0]:.2f}, y={self._goal[1]:.2f}, z={self._goal[2]:.2f}'
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom = msg

    def _timer_cb(self) -> None:
        cmd = Twist()
        if self._odom is None:
            self._pub.publish(cmd)
            return

        vel_world = (0.0, 0.0, 0.0)
        if self._goal is None or self._speed <= 0.0:
            self._publish_ref(vel_world)
            self._pub.publish(cmd)
            return

        pose = self._odom.pose.pose
        dx = self._goal[0] - pose.position.x
        dy = self._goal[1] - pose.position.y
        dz = self._goal[2] - pose.position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance <= self._goal_tolerance:
            self._publish_ref(vel_world)
            self._pub.publish(cmd)
            return

        scale = self._speed / distance
        vel_world = (dx * scale, dy * scale, dz * scale)
        vel_out = vel_world

        if self._body_frame_output:
            q = pose.orientation
            vel_out = _rotate_world_to_body(vel_world, (q.x, q.y, q.z, q.w))

        cmd.linear = Vector3(x=float(vel_out[0]), y=float(vel_out[1]), z=float(vel_out[2]))
        self._pub.publish(cmd)
        self._publish_ref(vel_world)

    def _publish_ref(self, vel_world: Tuple[float, float, float]) -> None:
        if not self._publish_horizon_ref or self._odom is None:
            return

        pose = self._odom.pose.pose
        q = pose.orientation
        current_yaw = _yaw_from_quat((q.x, q.y, q.z, q.w))
        horizontal_speed = math.hypot(vel_world[0], vel_world[1])
        if self._align_yaw_to_velocity and horizontal_speed > 1e-3:
            yaw = math.atan2(vel_world[1], vel_world[0])
        else:
            yaw = current_yaw
        quat = _quat_from_yaw(yaw)

        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self._world_frame)
        traj = MultiDOFJointTrajectory(header=header)
        path = Path(header=header)

        dt = self._horizon_time / float(self._horizon_steps)
        for i in range(self._horizon_steps + 1):
            t = dt * float(i)
            position = Vector3(
                x=float(pose.position.x + vel_world[0] * t),
                y=float(pose.position.y + vel_world[1] * t),
                z=float(pose.position.z + vel_world[2] * t),
            )
            traj.points.append(MultiDOFJointTrajectoryPoint(
                transforms=[Transform(translation=position, rotation=quat)],
                velocities=[Twist(
                    linear=Vector3(
                        x=float(vel_world[0]),
                        y=float(vel_world[1]),
                        z=float(vel_world[2]),
                    )
                )],
                accelerations=[Twist()],
                time_from_start=Duration(
                    sec=int(t),
                    nanosec=int((t - int(t)) * 1e9),
                ),
            ))

            pose_msg = PoseStamped(header=header)
            pose_msg.pose.position = Point(x=position.x, y=position.y, z=position.z)
            pose_msg.pose.orientation = quat
            path.poses.append(pose_msg)

        self._horizon_pub.publish(traj)
        self._horizon_viz_pub.publish(path)


def main() -> None:
    rclpy.init()
    node = GoalVelocityCommander()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
