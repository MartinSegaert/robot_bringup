#!/usr/bin/env python3

"""Select GBPlanner when a lidar return is close to the sensor origin."""

from lidar_proximity import DurationThresholdFilter, has_point_within_distance
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Bool


class UseGbplannerSelector(Node):
    def __init__(self):
        super().__init__('use_gbplanner_selector')

        self.declare_parameter(
            'input_topic', '/rmf/lidar/points_downsampled'
        )
        self.declare_parameter('output_topic', '/use_gbplanner')
        self.declare_parameter('distance_threshold', 5.0)
        self.declare_parameter('duration_threshold', 1.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.distance_threshold = float(
            self.get_parameter('distance_threshold').value
        )
        self.duration_threshold = float(
            self.get_parameter('duration_threshold').value
        )
        if self.distance_threshold <= 0.0:
            raise ValueError('distance_threshold must be positive')
        self.state_filter = DurationThresholdFilter(
            initial_value=True,
            duration_threshold=self.duration_threshold,
        )

        # Retain the latest decision so late subscribers also receive the
        # default selection before the first point cloud arrives.
        output_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(Bool, output_topic, output_qos)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.pointcloud_callback, 10
        )

        self.publisher.publish(Bool(data=True))
        self.get_logger().info(
            f'Watching {input_topic} within {self.distance_threshold:.2f} m; '
            f'changes must persist for {self.duration_threshold:.2f} s; '
            f'publishing the decision on {output_topic}'
        )

    def pointcloud_callback(self, message):
        points = pc2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=False
        )
        observed_use_gbplanner = has_point_within_distance(
            points, self.distance_threshold
        )
        previous_value = self.state_filter.value
        use_gbplanner = self.state_filter.update(
            observed_use_gbplanner,
            self.get_clock().now().nanoseconds,
        )
        if use_gbplanner != previous_value:
            self.get_logger().info(
                f'use_gbplanner changed to {use_gbplanner}'
            )
        self.publisher.publish(Bool(data=use_gbplanner))


def main(args=None):
    rclpy.init(args=args)
    node = UseGbplannerSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
