#!/usr/bin/env python3

"""Select GBPlanner when a lidar return is close to the sensor origin."""

from lidar_proximity import has_point_within_distance
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

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.distance_threshold = float(
            self.get_parameter('distance_threshold').value
        )
        if self.distance_threshold <= 0.0:
            raise ValueError('distance_threshold must be positive')

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
            f'publishing the decision on {output_topic}'
        )

    def pointcloud_callback(self, message):
        points = pc2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=False
        )
        use_gbplanner = has_point_within_distance(
            points, self.distance_threshold
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
