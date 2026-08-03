#!/usr/bin/env python3
"""Helicopter-specific point cloud downsampler.

Extends pc_sparsify with TF-based timestamp correction needed for the
ArduPilot SITL setup, where Gazebo sim-time and ROS wall-clock are
different domains.  The published cloud is stamped with the timestamp
of the latest map→<robot>/base_link TF (broadcast by GzPoseBridgeNode
at wall-clock time) so that voxblox TF lookups always succeed.
"""

import numpy as np
from pointcloud_clearing import replace_non_returns
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
import rclpy.time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros

_CLOCK_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class PointCloudDownsampler(Node):
    def __init__(self):
        super().__init__('pointcloud_downsampler')

        # Parameters
        self.declare_parameter('input_topic', '/rmf/lidar/points')
        self.declare_parameter('output_topic', '/rmf/lidar/points_downsampled')
        self.declare_parameter('output_frame_id', '')  # if set, overrides Gazebo's frame_id
        self.declare_parameter('voxel_size', 0.1)  # 5cm voxel grid
        self.declare_parameter('skip_points', 5)     # Keep every Nth point (alternative method)
        self.declare_parameter('method', 'skip')    # 'voxel' or 'skip'
        self.declare_parameter('tf_parent_frame', 'map')
        self.declare_parameter('tf_child_frame', 'helicopter/base_link')
        self.declare_parameter('allow_clear', True)
        self.declare_parameter('max_ray_length_m', 20.0)
        self.declare_parameter('clearing_ray_margin_m', 0.1)
        self.declare_parameter('horizontal_min_angle', -np.pi)
        self.declare_parameter('horizontal_max_angle', np.pi)
        self.declare_parameter('vertical_min_angle', -0.7854)
        self.declare_parameter('vertical_max_angle', 1.5)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.skip_points = self.get_parameter('skip_points').value
        self.method = self.get_parameter('method').value
        self._tf_parent = self.get_parameter('tf_parent_frame').value
        self._tf_child = self.get_parameter('tf_child_frame').value
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

        # Subscriber and Publisher
        self.sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            10
        )

        self.pub = self.create_publisher(PointCloud2, output_topic, 10)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Offset (nanoseconds) = wall_ns - gz_sim_ns, updated from /clock.
        # Used to convert Gazebo sim timestamps to wall-clock time so TF
        # lookups (which run on wall clock via MAVROS) succeed.
        self._gz_to_wall_offset_ns: int | None = None
        self.create_subscription(Clock, '/clock', self._clock_cb, _CLOCK_QOS)

        self.get_logger().info(f'Downsampling point clouds from {input_topic} to {output_topic}')
        self.get_logger().info(
            f'Method: {self.method}, Voxel size: {self.voxel_size}, '
            f'Skip: {self.skip_points}'
        )
        self.get_logger().info(
            f'Non-return clearing: {self.allow_clear}, endpoint distance: '
            f'{self.max_ray_length_m + self.clearing_ray_margin_m:.2f} m'
        )

    def _clock_cb(self, msg: Clock) -> None:
        # self.get_clock().now() is preferred over time.time() — both are wall
        # clock when use_sim_time is false, but get_clock() is on the same
        # clock domain as odom_relay's TF timestamps.
        # wall_ns = int(time.time() * 1e9)  # alternative: Python system clock
        wall_ns = self.get_clock().now().nanoseconds
        gz_ns = msg.clock.sec * 10**9 + msg.clock.nanosec
        self._gz_to_wall_offset_ns = wall_ns - gz_ns

    def pointcloud_callback(self, msg):
        if self._gz_to_wall_offset_ns is None:
            return  # no clock offset yet — drop until /clock arrives
        if self.method == 'voxel':
            downsampled_msg = self.voxel_downsample(msg)
        else:
            downsampled_msg = self.skip_downsample(msg)

        # downsampled_msg.header.stamp = rclpy.time.Time(nanoseconds=corrected_ns).to_msg()
        # downsampled_msg.header.stamp = self.get_clock().now().to_msg()
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                self._tf_parent, self._tf_child, rclpy.time.Time())
            downsampled_msg.header.stamp = tf_stamped.header.stamp
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            downsampled_msg.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().warn('TF lookup failed, using wall clock time')

        downsampled_msg.header.stamp = self.get_clock().now().to_msg()
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
