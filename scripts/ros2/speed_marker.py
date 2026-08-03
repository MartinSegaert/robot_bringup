#!/usr/bin/env python3
"""Render a scalar speed topic as text attached to a robot TF frame."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker


class SpeedMarker(Node):
    """Convert ``std_msgs/Float32`` speed samples to an RViz text marker."""

    def __init__(self) -> None:
        super().__init__('speed_marker')

        self.declare_parameter('speed_topic', '/sdf_nmpc/output/speed')
        self.declare_parameter('marker_topic', '/uav/speed_marker')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('height', 2.0)
        self.declare_parameter('text_scale', 0.5)

        speed_topic = str(self.get_parameter('speed_topic').value)
        marker_topic = str(self.get_parameter('marker_topic').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._height = float(self.get_parameter('height').value)
        self._text_scale = max(float(self.get_parameter('text_scale').value), 0.01)

        self._publisher = self.create_publisher(Marker, marker_topic, 10)
        self.create_subscription(Float32, speed_topic, self._speed_cb, 10)

        self.get_logger().info(
            f'Displaying {speed_topic} as {marker_topic} in frame {self._frame_id}'
        )

    def _speed_cb(self, speed: Float32) -> None:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._frame_id
        marker.ns = 'uav_speed'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.z = self._height
        marker.pose.orientation.w = 1.0
        marker.scale.z = self._text_scale
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f'Speed: {speed.data:.2f} m/s'

        self._publisher.publish(marker)


def main() -> None:
    rclpy.init()
    node = SpeedMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
