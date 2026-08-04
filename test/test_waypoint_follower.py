from pathlib import Path as FilesystemPath
import sys
from types import SimpleNamespace

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter


SCRIPT_DIR = FilesystemPath(__file__).parents[1] / 'scripts' / 'ros2'
sys.path.insert(0, str(SCRIPT_DIR))

from waypoint_follower import WaypointFollower, parse_waypoints  # noqa: E402


def test_parse_waypoints_supports_yaw_and_quaternion():
    waypoints = parse_waypoints([
        {'name': 'yaw', 'x': 1, 'y': 2, 'z': 3, 'yaw_deg': 90},
        {'name': 'quat', 'x': 4, 'y': 5, 'qz': 2, 'qw': 2},
    ])

    assert waypoints[0]['name'] == 'yaw'
    assert waypoints[0]['qz'] == pytest.approx(2 ** -0.5)
    assert waypoints[0]['qw'] == pytest.approx(2 ** -0.5)
    assert waypoints[1]['qz'] == pytest.approx(2 ** -0.5)
    assert waypoints[1]['qw'] == pytest.approx(2 ** -0.5)


def test_parse_waypoints_rejects_invalid_entries():
    with pytest.raises(ValueError, match='must define x and y'):
        parse_waypoints([{'x': 1.0}])

    with pytest.raises(ValueError, match='both yaw and a quaternion'):
        parse_waypoints([{'x': 1.0, 'y': 2.0, 'yaw': 0.0, 'qw': 1.0}])


def test_autostart_publishes_and_advances_waypoints(tmp_path):
    waypoint_file = tmp_path / 'waypoints.yaml'
    waypoint_file.write_text(
        'frame_id: map\n'
        'inter_waypoint_delay: 0.01\n'
        'waypoints:\n'
        '  - name: first\n'
        '    x: 1.0\n'
        '    y: 0.0\n'
        '    z: 2.0\n'
        '  - name: second\n'
        '    x: 3.0\n'
        '    y: 0.0\n'
        '    z: 2.0\n',
        encoding='utf-8',
    )

    rclpy.init()
    follower = WaypointFollower(parameter_overrides=[
        Parameter('use_sim_time', value=False),
        Parameter('waypoint_file', value=str(waypoint_file)),
        Parameter('autostart', value=True),
        Parameter('reached_distance', value=0.2),
    ])
    published = []
    follower._goal_publisher = SimpleNamespace(publish=published.append)

    try:
        assert follower._running
        assert follower._send_pending

        follower._send_next_waypoint()
        assert len(published) == 1
        assert published[-1].header.frame_id == 'map'
        assert published[-1].pose.position.x == 1.0

        odometry = Odometry()
        odometry.pose.pose.position.x = 1.0
        odometry.pose.pose.position.z = 2.0
        follower._odometry_callback(odometry)
        follower._send_next_waypoint()
        assert len(published) == 2
        assert published[-1].pose.position.x == 3.0

        odometry.pose.pose.position.x = 3.0
        follower._odometry_callback(odometry)
        follower._send_next_waypoint()
        assert not follower._running
        assert follower._index == 2
    finally:
        follower._cancel_send_timer()
        follower.destroy_node()
        rclpy.shutdown()
