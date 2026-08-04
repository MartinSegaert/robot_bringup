#!/usr/bin/env python3
"""Periodically publish a collision-unaware path from the UAV to its goal."""

import math
from typing import Optional

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32


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


def closest_lidar_distance(point_cloud: PointCloud2) -> float:
    """Return the shortest finite range from the lidar origin."""
    field_names = {field.name for field in point_cloud.fields}
    if not {'x', 'y', 'z'}.issubset(field_names):
        return math.inf

    minimum_distance = math.inf
    for point in point_cloud2.read_points(
        point_cloud,
        field_names=['x', 'y', 'z'],
        skip_nans=True,
    ):
        x, y, z = (float(point[index]) for index in range(3))
        distance = math.sqrt(x * x + y * y + z * z)
        if math.isfinite(distance):
            minimum_distance = min(minimum_distance, distance)
    return minimum_distance


def interpolate_reference_speed(
    distance: float,
    min_distance: float,
    max_distance: float,
    min_speed: float,
    max_speed: float,
) -> float:
    """Map a clearance to speed using a clipped linear interpolation."""
    if max_distance <= min_distance:
        raise ValueError('max_distance must be greater than min_distance')
    if min_speed < 0.0 or max_speed < min_speed:
        raise ValueError('speeds must satisfy 0 <= min_speed <= max_speed')

    if not math.isfinite(distance) or distance >= max_distance:
        return max_speed
    if distance <= min_distance:
        return min_speed

    ratio = (distance - min_distance) / (max_distance - min_distance)
    return min_speed + ratio * (max_speed - min_speed)


def allowed_reference_speed(
    obstacle_distance: float,
    arrival_distance: float,
    min_distance: float,
    max_distance: float,
    min_speed: float,
    max_speed: float,
) -> float:
    """Return the speed allowed by the nearest obstacle or arrival point."""
    limiting_distance = min(obstacle_distance, arrival_distance)
    return interpolate_reference_speed(
        limiting_distance,
        min_distance,
        max_distance,
        min_speed,
        max_speed,
    )


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


def distance_to_path(position, path: Path) -> float:
    """Return the shortest 3D distance from a position to a path polyline."""
    if not path.poses:
        return math.inf

    if len(path.poses) == 1:
        point = path.poses[0].pose.position
        return math.sqrt(
            (position.x - point.x) ** 2
            + (position.y - point.y) ** 2
            + (position.z - point.z) ** 2
        )

    minimum_distance = math.inf
    for start_pose, end_pose in zip(path.poses, path.poses[1:]):
        start = start_pose.pose.position
        end = end_pose.pose.position
        segment_x = end.x - start.x
        segment_y = end.y - start.y
        segment_z = end.z - start.z
        segment_length_squared = (
            segment_x * segment_x
            + segment_y * segment_y
            + segment_z * segment_z
        )

        if segment_length_squared <= 1e-12:
            ratio = 0.0
        else:
            ratio = (
                (position.x - start.x) * segment_x
                + (position.y - start.y) * segment_y
                + (position.z - start.z) * segment_z
            ) / segment_length_squared
            ratio = min(1.0, max(0.0, ratio))

        closest_x = start.x + ratio * segment_x
        closest_y = start.y + ratio * segment_y
        closest_z = start.z + ratio * segment_z
        distance = math.sqrt(
            (position.x - closest_x) ** 2
            + (position.y - closest_y) ** 2
            + (position.z - closest_z) ** 2
        )
        minimum_distance = min(minimum_distance, distance)

    return minimum_distance


class StraightLineGlobalPlanner(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            'straight_line_global_planner',
            parameter_overrides=parameter_overrides,
        )

        self.declare_parameter('odometry_topic', '/rmf/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/gbplanner_path')
        self.declare_parameter('lidar_topic', '/rmf/lidar/points')
        self.declare_parameter(
            'reference_speed_topic', '/sdf_nmpc/reference_speed'
        )
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('segment_length', 1.0)
        self.declare_parameter('retrigger_distance', 1.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('min_distance', 1.0)
        self.declare_parameter('max_distance', 5.0)

        odometry_topic = str(self.get_parameter('odometry_topic').value)
        goal_topic = str(self.get_parameter('goal_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        lidar_topic = str(self.get_parameter('lidar_topic').value)
        reference_speed_topic = str(
            self.get_parameter('reference_speed_topic').value
        )
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._segment_length = float(
            self.get_parameter('segment_length').value
        )
        self._retrigger_distance = float(
            self.get_parameter('retrigger_distance').value
        )
        self._min_speed = float(self.get_parameter('min_speed').value)
        self._max_speed = float(self.get_parameter('max_speed').value)
        self._min_distance = float(self.get_parameter('min_distance').value)
        self._max_distance = float(self.get_parameter('max_distance').value)
        if self._segment_length <= 0.0 or not math.isfinite(
            self._segment_length
        ):
            raise ValueError('segment_length must be a finite positive number')
        if self._retrigger_distance <= 0.0 or not math.isfinite(
            self._retrigger_distance
        ):
            raise ValueError(
                'retrigger_distance must be a finite positive number'
            )
        speed_parameters = (
            self._min_speed,
            self._max_speed,
            self._min_distance,
            self._max_distance,
        )
        if not all(math.isfinite(value) for value in speed_parameters):
            raise ValueError('speed and distance limits must be finite')
        # Validate the interpolation limits once during startup.
        interpolate_reference_speed(
            self._min_distance,
            self._min_distance,
            self._max_distance,
            self._min_speed,
            self._max_speed,
        )

        self._odometry: Optional[Odometry] = None
        self._goal: Optional[PoseStamped] = None
        self._path: Optional[Path] = None
        self._goal_waiting_for_odometry = False
        self._closest_obstacle_distance = math.inf

        self._path_publisher = self.create_publisher(Path, path_topic, _PATH_QOS)
        self._reference_speed_publisher = self.create_publisher(
            Float32, reference_speed_topic, _PATH_QOS
        )
        self.create_subscription(
            Odometry, odometry_topic, self._odometry_callback, _SENSOR_QOS
        )
        self.create_subscription(PoseStamped, goal_topic, self._goal_callback, 10)
        self.create_subscription(
            PointCloud2,
            lidar_topic,
            self._point_cloud_callback,
            _SENSOR_QOS,
        )

        self.get_logger().info(
            f'Straight-line planner: {odometry_topic} + {goal_topic} -> '
            f'{path_topic}; {self._segment_length:.2f} m target segments, '
            f'retriggering after {self._retrigger_distance:.2f} m path deviation'
        )
        self.get_logger().info(
            f'Reference speed: {self._min_speed:.2f}-{self._max_speed:.2f} '
            f'm/s over {self._min_distance:.2f}-{self._max_distance:.2f} m; '
            f'{lidar_topic} -> {reference_speed_topic}'
        )

    def _odometry_callback(self, message: Odometry) -> None:
        self._odometry = message
        self._publish_reference_speed()
        if self._goal_waiting_for_odometry:
            self._goal_waiting_for_odometry = False
            self._publish_path()
            return

        if self._goal is None or self._path is None:
            return
        if not _position_is_finite(message.pose.pose):
            self.get_logger().error(
                'Cannot check path deviation from non-finite odometry'
            )
            return

        deviation = distance_to_path(message.pose.pose.position, self._path)
        if deviation > self._retrigger_distance:
            self.get_logger().info(
                f'UAV is {deviation:.2f} m from its path; replanning'
            )
            self._publish_path()

    def _goal_callback(self, message: PoseStamped) -> None:
        if not _position_is_finite(message.pose):
            self.get_logger().error('Ignoring goal with non-finite coordinates')
            return

        self._goal = message
        self._publish_reference_speed()
        if self._odometry is None:
            self._goal_waiting_for_odometry = True
            self.get_logger().info(
                'Goal received; waiting for the first odometry message'
            )
            return

        # Publish a new operator goal immediately.
        self._publish_path()

    def _point_cloud_callback(self, message: PointCloud2) -> None:
        self._closest_obstacle_distance = closest_lidar_distance(message)
        self._publish_reference_speed()

    def _arrival_distance(self) -> float:
        if self._odometry is None or self._goal is None:
            return math.inf
        current = self._odometry.pose.pose.position
        target = self._goal.pose.position
        return math.sqrt(
            (target.x - current.x) ** 2
            + (target.y - current.y) ** 2
            + (target.z - current.z) ** 2
        )

    def _publish_reference_speed(self) -> None:
        speed = allowed_reference_speed(
            self._closest_obstacle_distance,
            self._arrival_distance(),
            self._min_distance,
            self._max_distance,
            self._min_speed,
            self._max_speed,
        )
        self._reference_speed_publisher.publish(Float32(data=float(speed)))

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
        self._path = path

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
