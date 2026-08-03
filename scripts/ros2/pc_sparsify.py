#!/usr/bin/env python3

import numpy as np
from pointcloud_clearing import replace_non_returns
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class PointCloudDownsampler(Node):
    def __init__(self):
        super().__init__('pointcloud_downsampler')

        # Parameters
        self.declare_parameter('input_topic', '/rmf/lidar/points')
        self.declare_parameter('output_topic', '/rmf/lidar/points_downsampled')
        self.declare_parameter('voxel_size', 0.1)  # 5cm voxel grid
        self.declare_parameter('skip_points', 5)     # Keep every Nth point (alternative method)
        self.declare_parameter('method', 'skip')    # 'voxel' or 'skip'
        self.declare_parameter('allow_clear', True)
        self.declare_parameter('max_ray_length_m', 20.0)
        self.declare_parameter('clearing_ray_margin_m', 0.8)
        self.declare_parameter('horizontal_min_angle', -np.pi)
        self.declare_parameter('horizontal_max_angle', np.pi)
        self.declare_parameter('vertical_min_angle', -0.7854)
        self.declare_parameter('vertical_max_angle', 1.5)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.skip_points = self.get_parameter('skip_points').value
        self.method = self.get_parameter('method').value
        self.allow_clear = self.get_parameter('allow_clear').value
        self.max_ray_length_m = self.get_parameter('max_ray_length_m').value
        self.clearing_ray_margin_m = self.get_parameter('clearing_ray_margin_m').value
        self.horizontal_min_angle = self.get_parameter('horizontal_min_angle').value
        self.horizontal_max_angle = self.get_parameter('horizontal_max_angle').value
        self.vertical_min_angle = self.get_parameter('vertical_min_angle').value
        self.vertical_max_angle = self.get_parameter('vertical_max_angle').value

        if self.max_ray_length_m <= 0.0:
            raise ValueError('max_ray_length_m must be positive')
        if self.clearing_ray_margin_m <= 0.0:
            raise ValueError('clearing_ray_margin_m must be positive')
        if self.horizontal_min_angle >= self.horizontal_max_angle:
            raise ValueError(
                'horizontal_min_angle must be less than horizontal_max_angle'
            )
        if not (
            -np.pi / 2 <= self.vertical_min_angle
            < self.vertical_max_angle <= np.pi / 2
        ):
            raise ValueError(
                'vertical angles must be ordered and within [-pi/2, pi/2]'
            )

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
        self.get_logger().info(
            f'Non-return clearing: {self.allow_clear}, endpoint distance: '
            f'{self.max_ray_length_m + self.clearing_ray_margin_m:.2f} m'
        )
        self.get_logger().info(
            'Clearing-ray FOV [rad]: '
            f'azimuth [{self.horizontal_min_angle:.3f}, '
            f'{self.horizontal_max_angle:.3f}], elevation '
            f'[{self.vertical_min_angle:.3f}, {self.vertical_max_angle:.3f}]'
        )

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
        downsampled_points, replaced_count = self.replace_non_returns(
            downsampled_points, source_indices, msg
        )

        # Rebuild the records from msg.fields instead of retaining Gazebo's
        # padded NumPy dtype. Humble's create_cloud rejects that padded dtype.
        point_records = [tuple(point) for point in downsampled_points]
        downsampled_msg = pc2.create_cloud(msg.header, msg.fields, point_records)

        self.log_replaced_non_returns(replaced_count)
        return downsampled_msg

    def voxel_downsample(self, msg):
        """Downsample with a voxel grid, keeping one point per voxel."""
        # Read points
        points = pc2.read_points(
            msg, skip_nans=False, field_names=('x', 'y', 'z')
        ).copy()
        source_indices = np.arange(len(points))
        points, replaced_count = self.replace_non_returns(
            points, source_indices, msg
        )

        if len(points) == 0:
            return msg

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

        self.log_replaced_non_returns(replaced_count)
        return downsampled_msg

    def replace_non_returns(self, points, source_indices, msg):
        return replace_non_returns(
            points=points,
            source_indices=source_indices,
            width=msg.width,
            height=msg.height,
            allow_clear=self.allow_clear,
            max_ray_length_m=self.max_ray_length_m,
            clearing_ray_margin_m=self.clearing_ray_margin_m,
            horizontal_min_angle=self.horizontal_min_angle,
            horizontal_max_angle=self.horizontal_max_angle,
            vertical_min_angle=self.vertical_min_angle,
            vertical_max_angle=self.vertical_max_angle,
        )

    def log_replaced_non_returns(self, replaced_count):
        if replaced_count:
            self.get_logger().info(
                f'Converted {replaced_count} far non-return beams to clearing rays',
                throttle_duration_sec=2.0
            )


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
