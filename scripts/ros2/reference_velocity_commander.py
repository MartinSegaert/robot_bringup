#!/usr/bin/env python3
"""Publish an obstacle- and waypoint-limited scalar reference speed."""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Float32

from waypoint_velocity_control import distance_speed_limit


_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class ReferenceVelocityCommander(Node):
    """Generate a scalar speed reference for the latest waypoint."""

    def __init__(self, **kwargs):
        super().__init__('reference_velocity_commander', **kwargs)

        defaults = {
            'waypoint_topic': '/current_waypoint',
            'odometry_topic': '/rmf/odom',
            'obstacle_topic': '/rmf/lidar/points_downsampled',
            'ref_vel_topic': '/rmf/ref_vel',
            'publish_rate': 20.0,
            'min_distance_obstacle': 1.0,
            'max_distance_obstacle': 5.0,
            'min_speed_obstacle': 0.2,
            'max_speed_obstacle': 2.0,
            'min_distance_waypoint': 1.0,
            'max_distance_waypoint': 5.0,
            'min_speed_waypoint': 0.2,
            'max_speed_waypoint': 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._obstacle_limits = self._read_limits('obstacle')
        self._waypoint_limits = self._read_limits('waypoint')
        publish_rate = float(self.get_parameter('publish_rate').value)
        self._validate_parameters(publish_rate)

        self._waypoint = None
        self._odometry = None
        self._obstacle_distance = math.inf

        waypoint_topic = str(self.get_parameter('waypoint_topic').value)
        odometry_topic = str(self.get_parameter('odometry_topic').value)
        obstacle_topic = str(self.get_parameter('obstacle_topic').value)
        ref_vel_topic = str(self.get_parameter('ref_vel_topic').value)

        self._publisher = self.create_publisher(Float32, ref_vel_topic, 10)
        self.create_subscription(PoseStamped, waypoint_topic, self._waypoint_cb, 10)
        self.create_subscription(
            Odometry, odometry_topic, self._odometry_cb, _SENSOR_QOS
        )
        self.create_subscription(
            PointCloud2, obstacle_topic, self._obstacle_cb, _SENSOR_QOS
        )
        self.create_timer(1.0 / publish_rate, self._timer_cb)

        self.get_logger().info(
            f'Publishing scalar reference speed on {ref_vel_topic}; waypoint: '
            f'{waypoint_topic}, odometry: {odometry_topic}, obstacles: '
            f'{obstacle_topic}'
        )

    def _read_limits(self, profile):
        return (
            float(self.get_parameter(f'min_distance_{profile}').value),
            float(self.get_parameter(f'max_distance_{profile}').value),
            float(self.get_parameter(f'min_speed_{profile}').value),
            float(self.get_parameter(f'max_speed_{profile}').value),
        )

    def _validate_parameters(self, publish_rate):
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')
        for profile, limits in (
            ('obstacle', self._obstacle_limits),
            ('waypoint', self._waypoint_limits),
        ):
            min_distance, max_distance, min_speed, max_speed = limits
            if min_distance < 0.0 or max_distance <= min_distance:
                raise ValueError(
                    f'{profile} distances must satisfy '
                    '0 <= min_distance < max_distance'
                )
            if min_speed <= 0.0 or max_speed < min_speed:
                raise ValueError(
                    f'{profile} speeds must satisfy '
                    '0 < min_speed <= max_speed'
                )

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
        waypoint_distance = math.inf
        if self._waypoint is not None and self._odometry is not None:
            position = self._odometry.pose.pose.position
            waypoint_distance = math.sqrt(
                (self._waypoint.x - position.x) ** 2
                + (self._waypoint.y - position.y) ** 2
                + (self._waypoint.z - position.z) ** 2
            )

        obstacle_speed = distance_speed_limit(
            self._obstacle_distance, *self._obstacle_limits
        )
        waypoint_speed = distance_speed_limit(
            waypoint_distance, *self._waypoint_limits
        )
        self._publisher.publish(Float32(data=min(obstacle_speed, waypoint_speed)))


def main(args=None):
    rclpy.init(args=args)
    node = ReferenceVelocityCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
