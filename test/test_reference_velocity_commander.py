import math
from pathlib import Path
import sys
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.duration import Duration
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
        Parameter('max_acceleration', value=1.0),
        Parameter('min_speed', value=2.0),
        Parameter('max_speed', value=4.0),
        Parameter('min_distance', value=12.0),
        Parameter('max_distance', value=20.0),
        Parameter('waypoint_arrival_speed', value=1.0),
        Parameter('waypoint_arrival_distance', value=5.0),
        Parameter('waypoint_tolerance', value=1.0),
    ], context=context)


def test_publishes_scalar_obstacle_limited_reference_speed():
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
        node._obstacle_distance = 16.0
        node._speed = 3.0
        node._last_update = node.get_clock().now() - Duration(seconds=1.0)

        node._timer_cb()

        assert len(published) == 1
        assert published[0].data == 3.0
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()


def test_slews_reference_to_zero_inside_waypoint_tolerance():
    context = Context()
    context.init(initialize_logging=False)
    node = None

    try:
        node = _make_node(context)
        published = []
        node._publisher = SimpleNamespace(publish=published.append)

        waypoint = PoseStamped()
        waypoint.pose.position.x = 0.5
        node._waypoint_cb(waypoint)
        node._odometry_cb(Odometry())
        node._speed = 3.0
        node._last_update = node.get_clock().now() - Duration(seconds=0.5)

        node._timer_cb()

        assert math.isclose(published[0].data, 2.5, rel_tol=0.0, abs_tol=1e-3)
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()
