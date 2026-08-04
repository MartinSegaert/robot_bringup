#!/usr/bin/env python3
"""Publish a configured sequence of ROS 2 target poses."""

import math
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import Trigger


_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

_GOAL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_DISCOVERY_RETRY_INTERVAL = 0.2


def _number(value, field: str, waypoint_index: int) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(
            f"waypoint {waypoint_index + 1} field '{field}' must be a number"
        )
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(
            f"waypoint {waypoint_index + 1} field '{field}' must be finite"
        )
    return result


def parse_waypoints(raw_waypoints) -> list[dict]:
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("'waypoints' must be a non-empty YAML list")

    parsed_waypoints = []
    for index, waypoint in enumerate(raw_waypoints):
        if not isinstance(waypoint, dict):
            raise ValueError(f'waypoint {index + 1} must be a YAML mapping')
        if 'x' not in waypoint or 'y' not in waypoint:
            raise ValueError(f'waypoint {index + 1} must define x and y')

        parsed = {
            'name': str(waypoint.get('name', f'waypoint_{index + 1}')),
            'x': _number(waypoint['x'], 'x', index),
            'y': _number(waypoint['y'], 'y', index),
            'z': _number(waypoint.get('z', 0.0), 'z', index),
        }

        quaternion_fields = ('qx', 'qy', 'qz', 'qw')
        has_quaternion = any(field in waypoint for field in quaternion_fields)
        has_yaw = 'yaw' in waypoint or 'yaw_deg' in waypoint
        if has_quaternion and has_yaw:
            raise ValueError(
                f'waypoint {index + 1} cannot define both yaw and a quaternion'
            )

        if has_quaternion:
            parsed.update({
                field: _number(
                    waypoint.get(field, 1.0 if field == 'qw' else 0.0),
                    field,
                    index,
                )
                for field in quaternion_fields
            })
            norm = math.sqrt(
                sum(parsed[field] ** 2 for field in quaternion_fields)
            )
            if norm < 1e-9:
                raise ValueError(
                    f'waypoint {index + 1} quaternion cannot be zero'
                )
            for field in quaternion_fields:
                parsed[field] /= norm
        else:
            if 'yaw_deg' in waypoint:
                yaw = math.radians(
                    _number(waypoint['yaw_deg'], 'yaw_deg', index)
                )
            else:
                yaw = _number(waypoint.get('yaw', 0.0), 'yaw', index)
            parsed.update({
                'qx': 0.0,
                'qy': 0.0,
                'qz': math.sin(yaw / 2.0),
                'qw': math.cos(yaw / 2.0),
            })

        parsed_waypoints.append(parsed)

    return parsed_waypoints


def load_waypoint_config(path: str) -> tuple[dict, list[dict]]:
    waypoint_path = Path(path).expanduser()
    if not waypoint_path.is_file():
        raise ValueError(f'waypoint file does not exist: {waypoint_path}')

    with waypoint_path.open('r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f'{waypoint_path} must contain a YAML mapping')

    return config, parse_waypoints(config.get('waypoints'))


class WaypointFollower(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            'waypoint_follower',
            parameter_overrides=parameter_overrides,
        )

        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('autostart', False)
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('odometry_topic', '/rmf/odom')
        self.declare_parameter('reached_distance', 2.0)

        waypoint_file = str(self.get_parameter('waypoint_file').value)
        goal_topic = str(self.get_parameter('goal_topic').value)
        odometry_topic = str(self.get_parameter('odometry_topic').value)
        reached_distance = float(self.get_parameter('reached_distance').value)
        if reached_distance <= 0.0 or not math.isfinite(reached_distance):
            raise ValueError('reached_distance must be a finite positive number')

        config, self._waypoints = load_waypoint_config(waypoint_file)
        self._frame_id = str(config.get('frame_id', 'map'))
        self._inter_waypoint_delay = float(
            config.get('inter_waypoint_delay', 0.5)
        )
        if self._inter_waypoint_delay < 0.0 or not math.isfinite(
            self._inter_waypoint_delay
        ):
            raise ValueError(
                'inter_waypoint_delay must be a finite nonnegative number'
            )
        self._reached_distance = reached_distance

        self._index = 0
        self._running = False
        self._waiting_for_reach = False
        self._send_pending = False
        self._send_timer = None
        self._position = None
        self._waiting_for_goal_subscriber = False

        self._goal_publisher = self.create_publisher(
            PoseStamped, goal_topic, _GOAL_QOS
        )
        self.create_subscription(
            Odometry, odometry_topic, self._odometry_callback, _SENSOR_QOS
        )
        self.create_service(Trigger, '~/start', self._start_callback)

        self.get_logger().info(
            f'Waypoint follower ready with {len(self._waypoints)} waypoint(s); '
            f'publishing ROS 2 goals on {goal_topic}; start service is '
            f'{self.get_name()}/start'
        )
        if bool(self.get_parameter('autostart').value):
            self._start_mission(initial_delay=0.5)

    def _start_callback(self, _request, response):
        if self._running or self._waiting_for_reach or self._send_pending:
            response.success = False
            response.message = 'mission is already running'
            return response

        if self._index >= len(self._waypoints):
            self._index = 0

        self._start_mission(initial_delay=0.0)
        response.success = True
        response.message = 'waypoint mission started'
        return response

    def _start_mission(self, initial_delay: float) -> None:
        self._running = True
        self.get_logger().info('Waypoint mission started')
        self._schedule_send(initial_delay)

    def _schedule_send(self, delay: float) -> None:
        self._send_pending = True
        self._send_timer = self.create_timer(
            max(delay, 1e-3), self._send_next_waypoint
        )

    def _cancel_send_timer(self) -> None:
        if self._send_timer is not None:
            self._send_timer.cancel()
            self.destroy_timer(self._send_timer)
            self._send_timer = None

    def _send_next_waypoint(self) -> None:
        self._cancel_send_timer()
        self._send_pending = False
        if not self._running:
            return

        if self._index >= len(self._waypoints):
            self._running = False
            self.get_logger().info(
                'Waypoint mission completed: all waypoints reached'
            )
            return

        # A volatile ROS 2 subscription does not receive a transient-local
        # sample that was published before DDS endpoint discovery completed.
        # Wait for the planner to match so the first (and every later) goal is
        # guaranteed to be delivered as a live sample.
        if self._goal_publisher.get_subscription_count() == 0:
            if not self._waiting_for_goal_subscriber:
                self.get_logger().info(
                    'Waiting for a subscriber on the goal topic'
                )
                self._waiting_for_goal_subscriber = True
            self._schedule_send(_DISCOVERY_RETRY_INTERVAL)
            return

        if self._waiting_for_goal_subscriber:
            self.get_logger().info(
                'Goal subscriber discovered; publishing waypoint'
            )
            self._waiting_for_goal_subscriber = False

        waypoint = self._waypoints[self._index]
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._frame_id
        goal.pose.position.x = waypoint['x']
        goal.pose.position.y = waypoint['y']
        goal.pose.position.z = waypoint['z']
        goal.pose.orientation.x = waypoint['qx']
        goal.pose.orientation.y = waypoint['qy']
        goal.pose.orientation.z = waypoint['qz']
        goal.pose.orientation.w = waypoint['qw']

        self._waiting_for_reach = True
        self._goal_publisher.publish(goal)
        self.get_logger().info(
            f"Published target {self._index + 1}/{len(self._waypoints)} "
            f"'{waypoint['name']}': ({waypoint['x']:.2f}, "
            f"{waypoint['y']:.2f}, {waypoint['z']:.2f})"
        )

    def _odometry_callback(self, message: Odometry) -> None:
        self._position = message.pose.pose.position
        if not self._running or not self._waiting_for_reach:
            return

        waypoint = self._waypoints[self._index]
        distance = math.sqrt(
            (self._position.x - waypoint['x']) ** 2
            + (self._position.y - waypoint['y']) ** 2
            + (self._position.z - waypoint['z']) ** 2
        )
        if distance > self._reached_distance:
            return

        self._waiting_for_reach = False
        self.get_logger().info(
            f"Reached waypoint {self._index + 1}/{len(self._waypoints)} "
            f"'{waypoint['name']}' (distance {distance:.2f} m)"
        )
        self._index += 1
        self._schedule_send(self._inter_waypoint_delay)


def main() -> None:
    rclpy.init()
    node: Optional[WaypointFollower] = None
    try:
        node = WaypointFollower()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
