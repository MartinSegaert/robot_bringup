#!/usr/bin/env python3
"""Periodically publish a collision-unaware path from the UAV to its goal."""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

_PATH_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _position_is_finite(pose: Pose) -> bool:
    position = pose.position
    return all(
        math.isfinite(value)
        for value in (position.x, position.y, position.z)
    )


def _yaw_orientation(yaw: float) -> Quaternion:
    """Return a unit quaternion containing only the requested yaw."""
    half_yaw = 0.5 * yaw
    return Quaternion(z=math.sin(half_yaw), w=math.cos(half_yaw))


def build_straight_path(
    odometry: Odometry,
    goal: PoseStamped,
    stamp,
    frame_id: str,
    segment_length: float,
) -> Path:
    """Build an equally spaced straight path consumed by the NMPC node."""
    if not math.isfinite(segment_length) or segment_length <= 0.0:
        raise ValueError('segment_length must be a finite positive number')

    path = Path()
    path.header.stamp = stamp
    path.header.frame_id = frame_id

    start = odometry.pose.pose.position
    target = goal.pose.position
    dx = target.x - start.x
    dy = target.y - start.y
    dz = target.z - start.z
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)

    # Pick the closest positive integer segment count, then interpolate using
    # that count so all segments have exactly the same length.
    segment_count = max(1, math.floor(distance / segment_length + 0.5))
    orientation = _yaw_orientation(math.atan2(dy, dx))

    for index in range(segment_count + 1):
        ratio = index / segment_count
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = (1.0 - ratio) * start.x + ratio * target.x
        pose.pose.position.y = (1.0 - ratio) * start.y + ratio * target.y
        pose.pose.position.z = (1.0 - ratio) * start.z + ratio * target.z
        pose.pose.orientation = orientation
        path.poses.append(pose)

    return path


class StraightLineGlobalPlanner(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            'straight_line_global_planner',
            parameter_overrides=parameter_overrides,
        )

        self.declare_parameter('odometry_topic', '/rmf/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/gbplanner_path')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('replan_interval', 2.0)
        self.declare_parameter('segment_length', 1.0)

        odometry_topic = str(self.get_parameter('odometry_topic').value)
        goal_topic = str(self.get_parameter('goal_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._replan_interval = float(
            self.get_parameter('replan_interval').value
        )
        self._segment_length = float(
            self.get_parameter('segment_length').value
        )
        if self._replan_interval <= 0.0 or not math.isfinite(
            self._replan_interval
        ):
            raise ValueError('replan_interval must be a finite positive number')
        if self._segment_length <= 0.0 or not math.isfinite(
            self._segment_length
        ):
            raise ValueError('segment_length must be a finite positive number')

        self._odometry: Optional[Odometry] = None
        self._goal: Optional[PoseStamped] = None
        self._goal_waiting_for_odometry = False

        self._path_publisher = self.create_publisher(Path, path_topic, _PATH_QOS)
        self.create_subscription(
            Odometry, odometry_topic, self._odometry_callback, _SENSOR_QOS
        )
        self.create_subscription(PoseStamped, goal_topic, self._goal_callback, 10)
        self.create_timer(self._replan_interval, self._timer_callback)

        self.get_logger().info(
            f'Straight-line planner: {odometry_topic} + {goal_topic} -> '
            f'{path_topic}; {self._segment_length:.2f} m target segments, '
            f'replanning every {self._replan_interval:.2f} s'
        )

    def _odometry_callback(self, message: Odometry) -> None:
        self._odometry = message
        if self._goal_waiting_for_odometry:
            self._goal_waiting_for_odometry = False
            self._publish_path()

    def _goal_callback(self, message: PoseStamped) -> None:
        if not _position_is_finite(message.pose):
            self.get_logger().error('Ignoring goal with non-finite coordinates')
            return

        self._goal = message
        if self._odometry is None:
            self._goal_waiting_for_odometry = True
            self.get_logger().info(
                'Goal received; waiting for the first odometry message'
            )
            return

        # Do not make a new operator goal wait for the periodic timer.
        self._publish_path()

    def _timer_callback(self) -> None:
        self._publish_path()

    def _publish_path(self) -> None:
        if self._odometry is None or self._goal is None:
            return
        if not _position_is_finite(self._odometry.pose.pose):
            self.get_logger().error(
                'Cannot plan from odometry with non-finite coordinates'
            )
            return

        path = build_straight_path(
            self._odometry,
            self._goal,
            self.get_clock().now().to_msg(),
            self._frame_id,
            self._segment_length,
        )
        self._path_publisher.publish(path)

        start = path.poses[0].pose.position
        target = path.poses[-1].pose.position
        distance = math.sqrt(
            (target.x - start.x) ** 2
            + (target.y - start.y) ** 2
            + (target.z - start.z) ** 2
        )
        self.get_logger().info(
            f'Published {distance:.2f} m straight path with '
            f'{len(path.poses) - 1} equal segments from '
            f'({start.x:.2f}, {start.y:.2f}, {start.z:.2f}) to '
            f'({target.x:.2f}, {target.y:.2f}, {target.z:.2f})'
        )


def main() -> None:
    rclpy.init()
    node = StraightLineGlobalPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
