from pathlib import Path
import sys
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.parameter import Parameter


SCRIPT_DIR = Path(__file__).parents[1] / 'scripts' / 'ros2'
sys.path.insert(0, str(SCRIPT_DIR))

from reference_velocity_commander import ReferenceVelocityCommander  # noqa: E402


def _make_node(context):
    return ReferenceVelocityCommander(parameter_overrides=[
        Parameter('waypoint_topic', value='/test/reference_velocity/waypoint'),
        Parameter('odometry_topic', value='/test/reference_velocity/odometry'),
        Parameter('obstacle_topic', value='/test/reference_velocity/obstacles'),
        Parameter('ref_vel_topic', value='/test/reference_velocity/output'),
        Parameter('min_distance_obstacle', value=12.0),
        Parameter('max_distance_obstacle', value=20.0),
        Parameter('min_speed_obstacle', value=2.0),
        Parameter('max_speed_obstacle', value=4.0),
        Parameter('min_distance_waypoint', value=5.0),
        Parameter('max_distance_waypoint', value=15.0),
        Parameter('min_speed_waypoint', value=1.0),
        Parameter('max_speed_waypoint', value=3.0),
    ], context=context)


def test_publishes_obstacle_limited_speed():
    context = Context()
    context.init(initialize_logging=False)
    node = None

    try:
        node = _make_node(context)
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


def test_publishes_waypoint_limited_speed():
    context = Context()
    context.init(initialize_logging=False)
    node = None

    try:
        node = _make_node(context)
        published = []
        node._publisher = SimpleNamespace(publish=published.append)

        waypoint = PoseStamped()
        waypoint.pose.position.x = 10.0
        node._waypoint_cb(waypoint)
        node._odometry_cb(Odometry())
        node._obstacle_distance = 100.0

        node._timer_cb()

        assert published[0].data == 2.0
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()


def test_publishes_waypoint_minimum_at_setpoint():
    context = Context()
    context.init(initialize_logging=False)
    node = None

    try:
        node = _make_node(context)
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


def test_publishes_lowest_maximum_without_distance_data():
    context = Context()
    context.init(initialize_logging=False)
    node = None

    try:
        node = _make_node(context)
        published = []
        node._publisher = SimpleNamespace(publish=published.append)
        node._timer_cb()

        assert published[0].data == 3.0
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()
