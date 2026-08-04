import math
from pathlib import Path as FilesystemPath
import sys
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


SCRIPT_DIR = FilesystemPath(__file__).parents[1] / 'scripts' / 'ros2'
sys.path.insert(0, str(SCRIPT_DIR))

from straight_line_global_planner import (  # noqa: E402
    allowed_reference_speed,
    build_straight_path,
    closest_lidar_distance,
    distance_to_path,
    interpolate_reference_speed,
    StraightLineGlobalPlanner,
)


def test_path_connects_latest_odometry_to_goal():
    odometry = Odometry()
    odometry.pose.pose.position.x = 2.0
    odometry.pose.pose.position.y = -1.0
    odometry.pose.pose.position.z = 3.0
    odometry.pose.pose.orientation.w = 2.0

    goal = PoseStamped()
    goal.pose.position.x = 10.0
    goal.pose.position.y = 4.0
    goal.pose.position.z = 2.5

    path = build_straight_path(
        odometry, goal, Time(sec=42), 'map', segment_length=2.0
    )

    assert isinstance(path, Path)
    assert path.header.frame_id == 'map'
    assert path.header.stamp.sec == 42
    assert len(path.poses) == 6
    assert path.poses[0].pose.position.x == 2.0
    assert path.poses[0].pose.position.y == -1.0
    assert path.poses[-1].pose.position.x == 10.0
    assert path.poses[-1].pose.position.y == 4.0

    expected_yaw = math.atan2(5.0, 8.0)
    for pose in path.poses:
        assert math.isclose(
            pose.pose.orientation.z, math.sin(expected_yaw / 2.0)
        )
        assert math.isclose(
            pose.pose.orientation.w, math.cos(expected_yaw / 2.0)
        )


def test_rebuilding_path_uses_updated_odometry():
    odometry = Odometry()
    odometry.pose.pose.orientation.w = 1.0
    goal = PoseStamped()
    goal.pose.position.x = 20.0
    goal.pose.position.z = 2.0

    first_path = build_straight_path(
        odometry, goal, Time(sec=1), 'map', segment_length=2.0
    )

    odometry.pose.pose.position.x = 5.0
    odometry.pose.pose.position.y = -3.0
    second_path = build_straight_path(
        odometry, goal, Time(sec=3), 'map', segment_length=2.0
    )

    assert first_path.poses[0].pose.position.x == 0.0
    assert second_path.poses[0].pose.position.x == 5.0
    assert second_path.poses[0].pose.position.y == -3.0
    assert second_path.poses[-1].pose.position.x == 20.0


def test_segments_are_equal_and_close_to_requested_length():
    odometry = Odometry()
    goal = PoseStamped()
    goal.pose.position.x = 6.0
    goal.pose.position.y = 8.0

    path = build_straight_path(
        odometry, goal, Time(), 'map', segment_length=3.0
    )

    # A 10 m path is closest to three 3.333 m segments, rather than four
    # 2.5 m segments.
    assert len(path.poses) == 4
    segment_lengths = []
    for start, end in zip(path.poses, path.poses[1:]):
        dx = end.pose.position.x - start.pose.position.x
        dy = end.pose.position.y - start.pose.position.y
        dz = end.pose.position.z - start.pose.position.z
        segment_lengths.append(math.sqrt(dx * dx + dy * dy + dz * dz))

    assert all(
        math.isclose(length, 10.0 / 3.0) for length in segment_lengths
    )


def test_distance_to_path_uses_nearest_point_on_segment():
    odometry = Odometry()
    goal = PoseStamped()
    goal.pose.position.x = 10.0
    path = build_straight_path(
        odometry, goal, Time(), 'map', segment_length=2.0
    )

    position = PoseStamped().pose.position
    position.x = 4.5
    position.y = 3.0
    position.z = 4.0

    assert math.isclose(distance_to_path(position, path), 5.0)


def test_reference_speed_is_clipped_and_linearly_interpolated():
    limits = (1.0, 5.0, 0.5, 4.5)

    assert interpolate_reference_speed(0.2, *limits) == 0.5
    assert interpolate_reference_speed(1.0, *limits) == 0.5
    assert interpolate_reference_speed(3.0, *limits) == 2.5
    assert interpolate_reference_speed(5.0, *limits) == 4.5
    assert interpolate_reference_speed(math.inf, *limits) == 4.5


def test_nearest_obstacle_or_arrival_point_limits_reference_speed():
    # The obstacle is far away, but the 2 m arrival distance limits the speed.
    assert math.isclose(
        allowed_reference_speed(8.0, 2.0, 1.0, 5.0, 1.0, 5.0),
        2.0,
    )
    # The arrival point is far away, but a close obstacle has the same effect.
    assert math.isclose(
        allowed_reference_speed(2.0, 8.0, 1.0, 5.0, 1.0, 5.0),
        2.0,
    )


def test_closest_lidar_distance_ignores_non_finite_points():
    cloud = point_cloud2.create_cloud_xyz32(
        Header(frame_id='lidar_link'),
        [
            (3.0, 4.0, 0.0),
            (math.nan, 0.0, 0.0),
            (1.0, 2.0, 2.0),
        ],
    )

    assert math.isclose(closest_lidar_distance(cloud), 3.0)


def test_deviation_retriggers_planner_from_latest_odometry():
    rclpy.init()
    planner = StraightLineGlobalPlanner(parameter_overrides=[
        Parameter('use_sim_time', value=False),
        Parameter('segment_length', value=2.0),
        Parameter('retrigger_distance', value=1.0),
        Parameter('odometry_topic', value='/test/planner/odometry'),
        Parameter('goal_topic', value='/test/planner/goal'),
        Parameter('path_topic', value='/test/planner/path'),
        Parameter('lidar_topic', value='/test/planner/lidar'),
        Parameter(
            'reference_speed_topic', value='/test/planner/reference_speed'
        ),
        Parameter('min_speed', value=1.0),
        Parameter('max_speed', value=5.0),
        Parameter('min_distance', value=1.0),
        Parameter('max_distance', value=5.0),
    ])
    published = []
    published_speeds = []
    planner._path_publisher = SimpleNamespace(publish=published.append)
    planner._reference_speed_publisher = SimpleNamespace(
        publish=published_speeds.append
    )

    try:
        odometry = Odometry()
        odometry.pose.pose.orientation.w = 1.0
        goal = PoseStamped()
        goal.pose.position.x = 20.0

        planner._odometry_callback(odometry)
        planner._goal_callback(goal)
        assert len(published) == 1
        assert published[-1].poses[0].pose.position.x == 0.0
        assert published_speeds[-1].data == 5.0

        obstacle_cloud = point_cloud2.create_cloud_xyz32(
            Header(frame_id='lidar_link'),
            [(2.0, 0.0, 0.0)],
        )
        planner._point_cloud_callback(obstacle_cloud)
        assert published_speeds[-1].data == 2.0

        # Progress along the existing path must not cause a replan.
        odometry.pose.pose.position.x = 6.0
        planner._odometry_callback(odometry)
        assert len(published) == 1

        # A deviation equal to the threshold does not retrigger either.
        odometry.pose.pose.position.y = 1.0
        planner._odometry_callback(odometry)
        assert len(published) == 1

        odometry.pose.pose.position.y = 1.01
        planner._odometry_callback(odometry)
        assert len(published) == 2
        assert published[-1].poses[0].pose.position.x == 6.0
        assert published[-1].poses[0].pose.position.y == 1.01
        assert published[-1].poses[-1].pose.position.x == 20.0
    finally:
        planner.destroy_node()
        rclpy.shutdown()
