from pathlib import Path as FilesystemPath
import math
import sys
from types import SimpleNamespace

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.parameter import Parameter


SCRIPT_DIR = FilesystemPath(__file__).parents[1] / 'scripts' / 'ros2'
sys.path.insert(0, str(SCRIPT_DIR))

from straight_line_global_planner import (  # noqa: E402
    StraightLineGlobalPlanner,
    build_straight_path,
    distance_to_path,
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


def test_deviation_retriggers_planner_from_latest_odometry():
    rclpy.init()
    planner = StraightLineGlobalPlanner(parameter_overrides=[
        Parameter('use_sim_time', value=False),
        Parameter('segment_length', value=2.0),
        Parameter('retrigger_distance', value=1.0),
        Parameter('odometry_topic', value='/test/planner/odometry'),
        Parameter('goal_topic', value='/test/planner/goal'),
        Parameter('path_topic', value='/test/planner/path'),
    ])
    published = []
    planner._path_publisher = SimpleNamespace(publish=published.append)

    try:
        odometry = Odometry()
        odometry.pose.pose.orientation.w = 1.0
        goal = PoseStamped()
        goal.pose.position.x = 20.0

        planner._odometry_callback(odometry)
        planner._goal_callback(goal)
        assert len(published) == 1
        assert published[-1].poses[0].pose.position.x == 0.0

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
