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

from waypoint_velocity_control import (
    obstacle_speed_limit,
    slew_speed,
    waypoint_speed_limit,
)


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
            'max_acceleration': 0.5,
            'min_speed': 0.2,
            'max_speed': 2.0,
            'min_distance': 1.0,
            'max_distance': 5.0,
            'waypoint_arrival_speed': 0.5,
            'waypoint_arrival_distance': 1.0,
            'waypoint_tolerance': 0.05,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._max_acceleration = float(
            self.get_parameter('max_acceleration').value
        )
        self._min_speed = float(self.get_parameter('min_speed').value)
        self._max_speed = float(self.get_parameter('max_speed').value)
        self._min_distance = float(self.get_parameter('min_distance').value)
        self._max_distance = float(self.get_parameter('max_distance').value)
        self._waypoint_arrival_speed = float(
            self.get_parameter('waypoint_arrival_speed').value
        )
        self._waypoint_arrival_distance = float(
            self.get_parameter('waypoint_arrival_distance').value
        )
        self._waypoint_tolerance = float(
            self.get_parameter('waypoint_tolerance').value
        )
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

    def _validate_parameters(self, publish_rate):
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')
        if self._max_acceleration <= 0.0:
            raise ValueError('max_acceleration must be positive')
        if self._min_speed < 0.0 or self._max_speed < self._min_speed:
            raise ValueError('speeds must satisfy 0 <= min_speed <= max_speed')
        if self._max_distance <= self._min_distance or self._min_distance < 0.0:
            raise ValueError('distances must satisfy 0 <= min_distance < max_distance')
        if self._waypoint_tolerance < 0.0:
            raise ValueError('waypoint_tolerance must be non-negative')
        if not 0.0 <= self._waypoint_arrival_speed <= self._max_speed:
            raise ValueError(
                'waypoint_arrival_speed must be between zero and max_speed'
            )
        if self._waypoint_arrival_distance < self._waypoint_tolerance:
            raise ValueError(
                'waypoint_arrival_distance must be at least waypoint_tolerance'
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
        now = self.get_clock().now()
        dt = max((now - self._last_update).nanoseconds * 1e-9, 0.0)
        self._last_update = now

        target_speed = 0.0
        if self._waypoint is not None and self._odometry is not None:
            position = self._odometry.pose.pose.position
            distance = math.sqrt(
                (self._waypoint.x - position.x) ** 2
                + (self._waypoint.y - position.y) ** 2
                + (self._waypoint.z - position.z) ** 2
            )

            if distance > self._waypoint_tolerance:
                obstacle_limit = obstacle_speed_limit(
                    self._obstacle_distance,
                    self._min_distance,
                    self._max_distance,
                    self._min_speed,
                    self._max_speed,
                )
                waypoint_limit = waypoint_speed_limit(
                    distance,
                    self._max_acceleration,
                    self._max_speed,
                    self._waypoint_arrival_distance,
                    self._waypoint_arrival_speed,
                )
                target_speed = min(obstacle_limit, waypoint_limit)

        self._speed = slew_speed(
            self._speed, target_speed, self._max_acceleration, dt
        )
        self._publisher.publish(Float32(data=float(self._speed)))


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
