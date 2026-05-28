#!/usr/bin/env python3
"""Publish nav_msgs/Odometry by looking up a TF between two frames.

Usage:
  tf_to_odom.py <parent_frame> <child_frame> <output_topic> [rate_hz]

Defaults:
  parent_frame  = map
  child_frame   = helicopter/base_link
  output_topic  = /helicopter/odom
  rate_hz       = 50.0
"""

import sys
import rclpy
from rclpy.node import Node
import tf2_ros
from nav_msgs.msg import Odometry


class TfToOdom(Node):
    def __init__(self, parent_frame: str, child_frame: str, output_topic: str, rate_hz: float):
        super().__init__('tf_to_odom')
        self._parent = parent_frame
        self._child = child_frame
        self._buf = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buf, self)
        self._pub = self.create_publisher(Odometry, output_topic, 10)
        self.create_timer(1.0 / rate_hz, self._cb)
        self._prev_ns = None
        self._prev_xyz = None
        self.get_logger().info(
            f'tf_to_odom: {parent_frame} → {child_frame} → {output_topic} @ {rate_hz} Hz'
        )

    def _cb(self) -> None:
        try:
            tf = self._buf.lookup_transform(self._parent, self._child, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        msg = Odometry()
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = self._parent
        msg.child_frame_id = self._child
        t = tf.transform.translation
        msg.pose.pose.position.x = t.x
        msg.pose.pose.position.y = t.y
        msg.pose.pose.position.z = t.z
        msg.pose.pose.orientation = tf.transform.rotation

        now_ns = self.get_clock().now().nanoseconds
        xyz = (t.x, t.y, t.z)
        if self._prev_ns is not None:
            dt = (now_ns - self._prev_ns) * 1e-9
            if dt > 0.0:
                msg.twist.twist.linear.x = (xyz[0] - self._prev_xyz[0]) / dt
                msg.twist.twist.linear.y = (xyz[1] - self._prev_xyz[1]) / dt
                msg.twist.twist.linear.z = (xyz[2] - self._prev_xyz[2]) / dt
        self._prev_ns = now_ns
        self._prev_xyz = xyz

        self._pub.publish(msg)


def main() -> None:
    parent = sys.argv[1] if len(sys.argv) > 1 else 'map'
    child  = sys.argv[2] if len(sys.argv) > 2 else 'helicopter/base_link'
    topic  = sys.argv[3] if len(sys.argv) > 3 else '/helicopter/odom'
    rate   = float(sys.argv[4]) if len(sys.argv) > 4 else 50.0
    rclpy.init()
    rclpy.spin(TfToOdom(parent, child, topic, rate))


if __name__ == '__main__':
    main()
