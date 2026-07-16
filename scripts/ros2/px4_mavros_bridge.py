#!/usr/bin/env python3
"""Connect the stack's acceleration/odometry topics to PX4 through MAVROS.

Acceleration commands are kept alive at ``setpoint_rate_hz`` while fresh.
Publishing stops after ``setpoint_timeout_s`` so PX4 can detect loss of the
offboard command stream instead of flying indefinitely on a stale command.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class Px4MavrosBridge(Node):
    def __init__(self) -> None:
        super().__init__('px4_mavros_bridge')

        self.declare_parameter('accel_input_topic', '/sdf_nmpc/cmd/acc')
        self.declare_parameter('setpoint_output_topic', '/mavros/setpoint_raw/local')
        self.declare_parameter('mavros_odom_topic', '/mavros/local_position/odom')
        self.declare_parameter('odom_output_topic', '/rmf/odom')
        self.declare_parameter('body_frame_id', 'rmf/base_link')
        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('setpoint_timeout_s', 0.5)

        accel_topic = self.get_parameter('accel_input_topic').value
        setpoint_topic = self.get_parameter('setpoint_output_topic').value
        mavros_odom_topic = self.get_parameter('mavros_odom_topic').value
        odom_topic = self.get_parameter('odom_output_topic').value
        self._body_frame_id = self.get_parameter('body_frame_id').value
        rate_hz = float(self.get_parameter('setpoint_rate_hz').value)
        self._timeout_s = float(self.get_parameter('setpoint_timeout_s').value)

        if rate_hz <= 2.0:
            raise ValueError('setpoint_rate_hz must be greater than PX4\'s 2 Hz offboard minimum')
        if self._timeout_s <= 0.0:
            raise ValueError('setpoint_timeout_s must be positive')

        self._setpoint_pub = self.create_publisher(PositionTarget, setpoint_topic, 10)
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.create_subscription(Twist, accel_topic, self._accel_callback, 10)
        self.create_subscription(Odometry, mavros_odom_topic, self._odom_callback, _SENSOR_QOS)

        self._last_setpoint: Optional[PositionTarget] = None
        self._last_command_ns: Optional[int] = None
        self._timed_out = False
        self.create_timer(1.0 / rate_hz, self._publish_setpoint)

        self.get_logger().info(
            f'Acceleration {accel_topic} -> {setpoint_topic}; '
            f'odometry {mavros_odom_topic} -> {odom_topic}'
        )

    def _accel_callback(self, command: Twist) -> None:
        target = PositionTarget()
        target.header.frame_id = self._body_frame_id
        target.coordinate_frame = PositionTarget.FRAME_BODY_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_YAW
        )
        target.acceleration_or_force.x = command.linear.x
        target.acceleration_or_force.y = command.linear.y
        target.acceleration_or_force.z = command.linear.z
        target.yaw_rate = command.angular.z

        self._last_setpoint = target
        self._last_command_ns = self.get_clock().now().nanoseconds
        self._timed_out = False

    def _publish_setpoint(self) -> None:
        if self._last_setpoint is None or self._last_command_ns is None:
            return

        now = self.get_clock().now()
        age_s = (now.nanoseconds - self._last_command_ns) * 1e-9
        if age_s > self._timeout_s:
            if not self._timed_out:
                self.get_logger().warning(
                    f'Acceleration command timed out after {age_s:.2f} s; stopping setpoints'
                )
                self._timed_out = True
            return

        self._last_setpoint.header.stamp = now.to_msg()
        self._setpoint_pub.publish(self._last_setpoint)

    def _odom_callback(self, odom: Odometry) -> None:
        self._odom_pub.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4MavrosBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
