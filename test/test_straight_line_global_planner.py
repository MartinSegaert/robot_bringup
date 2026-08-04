from pathlib import Path as FilesystemPath
import sys
import time
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

    path = build_straight_path(odometry, goal, Time(sec=42), 'map')

    assert isinstance(path, Path)
    assert path.header.frame_id == 'map'
    assert path.header.stamp.sec == 42
    assert len(path.poses) == 2
    assert path.poses[0].pose.position.x == 2.0
    assert path.poses[0].pose.position.y == -1.0
    assert path.poses[1].pose.position.x == 10.0
    assert path.poses[1].pose.position.y == 4.0
    assert path.poses[0].pose.orientation.w == 1.0
    assert path.poses[1].pose.orientation.w == 1.0


def test_rebuilding_path_uses_updated_odometry():
    odometry = Odometry()
    odometry.pose.pose.orientation.w = 1.0
    goal = PoseStamped()
    goal.pose.position.x = 20.0
    goal.pose.position.z = 2.0

    first_path = build_straight_path(odometry, goal, Time(sec=1), 'map')

    odometry.pose.pose.position.x = 5.0
    odometry.pose.pose.position.y = -3.0
    second_path = build_straight_path(odometry, goal, Time(sec=3), 'map')

    assert first_path.poses[0].pose.position.x == 0.0
    assert second_path.poses[0].pose.position.x == 5.0
    assert second_path.poses[0].pose.position.y == -3.0
    assert second_path.poses[1].pose.position.x == 20.0


def test_timer_republishes_from_latest_odometry():
    rclpy.init()
    planner = StraightLineGlobalPlanner(parameter_overrides=[
        Parameter('use_sim_time', value=False),
        Parameter('replan_interval', value=0.05),
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

        odometry.pose.pose.position.x = 6.0
        planner._odometry_callback(odometry)
        published.clear()

        deadline = time.monotonic() + 0.5
        while not published and time.monotonic() < deadline:
            rclpy.spin_once(planner, timeout_sec=0.05)

        assert len(published) == 1
        assert published[0].poses[0].pose.position.x == 6.0
        assert published[0].poses[1].pose.position.x == 20.0
    finally:
        planner.destroy_node()
        rclpy.shutdown()
