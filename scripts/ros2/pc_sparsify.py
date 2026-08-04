#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


def remove_non_returns(points):
    """Remove lidar records without finite Cartesian coordinates."""
    if not {'x', 'y', 'z'}.issubset(points.dtype.names or ()):
        raise ValueError('Point cloud must contain x, y, and z fields')

    xyz = np.column_stack((points['x'], points['y'], points['z']))
    return points[np.isfinite(xyz).all(axis=1)]


class PointCloudDownsampler(Node):
    def __init__(self):
        super().__init__('pointcloud_downsampler')

        # Parameters
        self.declare_parameter('input_topic', '/rmf/lidar/points')
        self.declare_parameter('output_topic', '/rmf/lidar/points_downsampled')
        self.declare_parameter('voxel_size', 0.1)  # 5cm voxel grid
        self.declare_parameter('skip_points', 5)     # Keep every Nth point (alternative method)
        self.declare_parameter('method', 'skip')    # 'voxel' or 'skip'

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.skip_points = self.get_parameter('skip_points').value
        self.method = self.get_parameter('method').value

        # Subscriber and Publisher
        self.sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            10
        )

        self.pub = self.create_publisher(PointCloud2, output_topic, 10)

        self.get_logger().info(f'Downsampling point clouds from {input_topic} to {output_topic}')
        self.get_logger().info(
            f'Method: {self.method}, Voxel size: {self.voxel_size}, '
            f'Skip: {self.skip_points}'
        )
        self.get_logger().info('Non-returning lidar beams are discarded')

    def pointcloud_callback(self, msg):
        if self.method == 'voxel':
            downsampled_msg = self.voxel_downsample(msg)
        else:
            downsampled_msg = self.skip_downsample(msg)

        self.pub.publish(downsampled_msg)

    def skip_downsample(self, msg):
        """Downsample by keeping every Nth point."""
        points = pc2.read_points(msg, skip_nans=False)
        source_indices = np.arange(0, len(points), self.skip_points)
        downsampled_points = points[source_indices].copy()
        downsampled_points = remove_non_returns(downsampled_points)

        # Rebuild the records from msg.fields instead of retaining Gazebo's
        # padded NumPy dtype. Humble's create_cloud rejects that padded dtype.
        point_records = [tuple(point) for point in downsampled_points]
        downsampled_msg = pc2.create_cloud(msg.header, msg.fields, point_records)

        return downsampled_msg

    def voxel_downsample(self, msg):
        """Downsample with a voxel grid, keeping one point per voxel."""
        # Read points
        points = pc2.read_points(
            msg, skip_nans=False, field_names=('x', 'y', 'z')
        ).copy()
        points = remove_non_returns(points)

        if len(points) == 0:
            return pc2.create_cloud_xyz32(msg.header, [])

        # Convert to numpy array
        xyz = np.column_stack((points['x'], points['y'], points['z']))

        # Compute voxel indices
        voxel_indices = np.floor(xyz / self.voxel_size).astype(np.int32)

        # Get unique voxels (keeps first occurrence)
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)

        # Get downsampled points
        downsampled_points = xyz[unique_indices]

        # Convert back to list of tuples
        downsampled_points_list = [tuple(p) for p in downsampled_points]

        # Create new PointCloud2 message
        downsampled_msg = pc2.create_cloud_xyz32(msg.header, downsampled_points_list)

        original_count = len(points)
        downsampled_count = len(downsampled_points_list)
        reduction = (1 - downsampled_count/original_count) * 100

        self.get_logger().info(
            f'Downsampled: {original_count} -> {downsampled_count} points '
            f'({reduction:.1f}% reduction)',
            throttle_duration_sec=2.0
        )

        return downsampled_msg


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDownsampler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
