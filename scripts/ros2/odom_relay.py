#!/usr/bin/env python3
"""Relay /mavros/local_position/odom → /<robot_name>/odom.

TF broadcasting (map → helicopter/base_link) has been moved to
GzPoseBridgeNode so the Gazebo pose update and TF update are always
triggered by the same odometry callback.
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class OdomRelay(Node):
    def __init__(self, input_topic: str, output_topic: str):
        super().__init__('odom_relay')
        self._pub = self.create_publisher(Odometry, output_topic, 10)
        self.create_subscription(Odometry, input_topic, self._cb, _SENSOR_QOS)
        self.get_logger().info(f'Relaying {input_topic} → {output_topic}')

    def _cb(self, msg: Odometry) -> None:
        self._pub.publish(msg)


def main():
    input_topic  = sys.argv[1] if len(sys.argv) > 1 else '/mavros/local_position/odom'
    output_topic = sys.argv[2] if len(sys.argv) > 2 else '/helicopter/odom'
    rclpy.init()
    rclpy.spin(OdomRelay(input_topic, output_topic))


if __name__ == '__main__':
    main()
