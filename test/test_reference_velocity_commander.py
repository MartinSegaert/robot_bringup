from pathlib import Path
import sys
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.parameter import Parameter
from std_msgs.msg import Bool


SCRIPT_DIR = Path(__file__).parents[1] / 'scripts' / 'ros2'
sys.path.insert(0, str(SCRIPT_DIR))

from reference_velocity_commander import ReferenceVelocityCommander  # noqa: E402


def _make_node(context, nmpc_config):
    return ReferenceVelocityCommander(parameter_overrides=[
        Parameter('waypoint_topic', value='/test/reference_velocity/waypoint'),
        Parameter('odometry_topic', value='/test/reference_velocity/odometry'),
        Parameter('obstacle_topic', value='/test/reference_velocity/obstacles'),
        Parameter('ref_vel_topic', value='/test/reference_velocity/output'),
        Parameter('nmpc_config', value=str(nmpc_config)),
        Parameter('min_distance_obstacle', value=12.0),
        Parameter('max_distance_obstacle', value=20.0),
        Parameter('min_speed_obstacle', value=2.0),
        Parameter('min_distance_waypoint', value=5.0),
        Parameter('max_distance_waypoint', value=15.0),
        Parameter('min_speed_waypoint', value=1.0),
        Parameter('max_speed', value=4.0),
    ], context=context)


def test_publishes_obstacle_limited_speed(tmp_path):
    context = Context()
    context.init(initialize_logging=False)
    node = None

    try:
        nmpc_config = tmp_path / 'nmpc.yaml'
        nmpc_config.write_text('ref:\n  vref: 3.5\n')
        node = _make_node(context, nmpc_config)
        published = []
        node._publisher = SimpleNamespace(publish=published.append)

        waypoint = PoseStamped()
        waypoint.pose.position.x = 100.0
        node._waypoint_cb(waypoint)
        node._odometry_cb(Odometry())
        node._obstacle_distance = 14.0

        node._timer_cb()

        assert len(published) == 1
        assert published[0].data == 2.5
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()


def test_publishes_waypoint_limited_speed(tmp_path):
    context = Context()
    context.init(initialize_logging=False)
    node = None

    try:
        nmpc_config = tmp_path / 'nmpc.yaml'
        nmpc_config.write_text('ref:\n  vref: 3.5\n')
        node = _make_node(context, nmpc_config)
        published = []
        node._publisher = SimpleNamespace(publish=published.append)

        waypoint = PoseStamped()
        waypoint.pose.position.x = 10.0
        node._waypoint_cb(waypoint)
        node._odometry_cb(Odometry())
        node._obstacle_distance = 100.0

        node._timer_cb()

        assert published[0].data == 2.5
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()


def test_publishes_waypoint_minimum_at_setpoint(tmp_path):
    context = Context()
    context.init(initialize_logging=False)
    node = None

    try:
        nmpc_config = tmp_path / 'nmpc.yaml'
        nmpc_config.write_text('ref:\n  vref: 3.5\n')
        node = _make_node(context, nmpc_config)
        published = []
        node._publisher = SimpleNamespace(publish=published.append)
        node._waypoint_cb(PoseStamped())
        node._odometry_cb(Odometry())

        node._timer_cb()

        assert published[0].data == 1.0
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()


def test_caps_speed_at_nmpc_vref_while_using_gbplanner(tmp_path):
    context = Context()
    context.init(initialize_logging=False)
    node = None

    try:
        nmpc_config = tmp_path / 'nmpc.yaml'
        nmpc_config.write_text('ref:\n  vref: 2.25\n')
        node = _make_node(context, nmpc_config)
        published = []
        node._publisher = SimpleNamespace(publish=published.append)
        node._use_gbplanner_cb(Bool(data=True))
        node._timer_cb()

        assert published[0].data == 2.25
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()


def test_allows_speed_above_nmpc_vref_when_not_using_gbplanner(tmp_path):
    context = Context()
    context.init(initialize_logging=False)
    node = None

    try:
        nmpc_config = tmp_path / 'nmpc.yaml'
        nmpc_config.write_text('ref:\n  vref: 2.25\n')
        node = _make_node(context, nmpc_config)
        published = []
        node._publisher = SimpleNamespace(publish=published.append)
        node._use_gbplanner_cb(Bool(data=False))
        node._timer_cb()

        assert published[0].data == 4.0
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()
