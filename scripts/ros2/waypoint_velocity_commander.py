#!/usr/bin/env python3
"""Publish acceleration- and obstacle-limited velocity toward a waypoint."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from waypoint_velocity_control import (
    clamp,
    obstacle_speed_limit,
    shortest_angular_distance,
    slew_speed,
    waypoint_speed_limit,
)


_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


def _yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
    )


def _world_to_body(vector, quaternion):
    """Rotate a world-frame vector into the body frame."""
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    vx, vy, vz = vector
    return (
        (1.0 - 2.0 * (y * y + z * z)) * vx
        + 2.0 * (x * y + z * w) * vy
        + 2.0 * (x * z - y * w) * vz,
        2.0 * (x * y - z * w) * vx
        + (1.0 - 2.0 * (x * x + z * z)) * vy
        + 2.0 * (y * z + x * w) * vz,
        2.0 * (x * z + y * w) * vx
        + 2.0 * (y * z - x * w) * vy
        + (1.0 - 2.0 * (x * x + y * y)) * vz,
    )


class WaypointVelocityCommander(Node):
    """Generate a Twist pointing at the latest PoseStamped waypoint."""

    def __init__(self):
        super().__init__('waypoint_velocity_commander')

        defaults = {
            'waypoint_topic': '/current_waypoint',
            'odometry_topic': '/rmf/odom',
            'obstacle_topic': '/rmf/lidar/points_downsampled',
            'cmd_vel_topic': '/cmd_vel',
            'publish_rate': 20.0,
            'max_acceleration': 0.5,
            'max_yaw_rate': 1.0,
            'yaw_gain': 1.5,
            'min_speed': 0.2,
            'max_speed': 2.0,
            'min_distance': 1.0,
            'max_distance': 5.0,
            'waypoint_tolerance': 0.05,
            'body_frame_output': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._max_acceleration = float(self.get_parameter('max_acceleration').value)
        self._max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self._yaw_gain = float(self.get_parameter('yaw_gain').value)
        self._min_speed = float(self.get_parameter('min_speed').value)
        self._max_speed = float(self.get_parameter('max_speed').value)
        self._min_distance = float(self.get_parameter('min_distance').value)
        self._max_distance = float(self.get_parameter('max_distance').value)
        self._waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self._body_frame_output = bool(self.get_parameter('body_frame_output').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self._validate_parameters(publish_rate)

        self._waypoint = None
        self._odometry = None
        self._obstacle_distance = math.inf
        self._speed = 0.0
        self._last_update = self.get_clock().now()

        waypoint_topic = str(self.get_parameter('waypoint_topic').value)
        odometry_topic = str(self.get_parameter('odometry_topic').value)
        obstacle_topic = str(self.get_parameter('obstacle_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self._publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(PoseStamped, waypoint_topic, self._waypoint_cb, 10)
        self.create_subscription(Odometry, odometry_topic, self._odometry_cb, _SENSOR_QOS)
        self.create_subscription(PointCloud2, obstacle_topic, self._obstacle_cb, _SENSOR_QOS)
        self.create_timer(1.0 / publish_rate, self._timer_cb)

        frame = 'body' if self._body_frame_output else 'world'
        self.get_logger().info(
            f'Publishing {frame}-frame commands on {cmd_vel_topic}; waypoint: '
            f'{waypoint_topic}, odometry: {odometry_topic}, obstacles: {obstacle_topic}'
        )

    def _validate_parameters(self, publish_rate):
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')
        if self._max_acceleration <= 0.0:
            raise ValueError('max_acceleration must be positive')
        if self._max_yaw_rate < 0.0 or self._yaw_gain < 0.0:
            raise ValueError('max_yaw_rate and yaw_gain must be non-negative')
        if self._min_speed < 0.0 or self._max_speed < self._min_speed:
            raise ValueError('speeds must satisfy 0 <= min_speed <= max_speed')
        if self._max_distance <= self._min_distance or self._min_distance < 0.0:
            raise ValueError('distances must satisfy 0 <= min_distance < max_distance')
        if self._waypoint_tolerance < 0.0:
            raise ValueError('waypoint_tolerance must be non-negative')

    def _waypoint_cb(self, message):
        self._waypoint = message.pose.position

    def _odometry_cb(self, message):
        self._odometry = message

    def _obstacle_cb(self, message):
        nearest_squared = math.inf
        for x, y, z in pc2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=True
        ):
            nearest_squared = min(nearest_squared, x * x + y * y + z * z)
        self._obstacle_distance = math.sqrt(nearest_squared)

    def _timer_cb(self):
        now = self.get_clock().now()
        dt = max((now - self._last_update).nanoseconds * 1e-9, 0.0)
        self._last_update = now
        command = Twist()

        if self._waypoint is None or self._odometry is None:
            self._speed = slew_speed(self._speed, 0.0, self._max_acceleration, dt)
            self._publisher.publish(command)
            return

        pose = self._odometry.pose.pose
        delta = (
            self._waypoint.x - pose.position.x,
            self._waypoint.y - pose.position.y,
            self._waypoint.z - pose.position.z,
        )
        distance = math.sqrt(sum(component * component for component in delta))

        if distance <= self._waypoint_tolerance:
            target_speed = 0.0
        else:
            obstacle_limit = obstacle_speed_limit(
                self._obstacle_distance,
                self._min_distance,
                self._max_distance,
                self._min_speed,
                self._max_speed,
            )
            waypoint_limit = waypoint_speed_limit(
                distance, self._max_acceleration, self._max_speed
            )
            target_speed = min(obstacle_limit, waypoint_limit)

        self._speed = slew_speed(
            self._speed, target_speed, self._max_acceleration, dt
        )
        if distance > 0.0 and self._speed > 0.0:
            velocity = tuple(self._speed * component / distance for component in delta)
            if self._body_frame_output:
                velocity = _world_to_body(velocity, pose.orientation)
            command.linear = Vector3(x=velocity[0], y=velocity[1], z=velocity[2])

        horizontal_distance = math.hypot(delta[0], delta[1])
        if horizontal_distance > self._waypoint_tolerance:
            desired_yaw = math.atan2(delta[1], delta[0])
            yaw_error = shortest_angular_distance(
                _yaw_from_quaternion(pose.orientation), desired_yaw
            )
            command.angular.z = clamp(
                self._yaw_gain * yaw_error, -self._max_yaw_rate, self._max_yaw_rate
            )

        self._publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointVelocityCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

