import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace


_STUB_MODULE_NAMES = (
    'rospy',
    'geometry_msgs',
    'geometry_msgs.msg',
    'nav_msgs',
    'nav_msgs.msg',
    'std_srvs',
    'std_srvs.srv',
)
_ORIGINAL_MODULES = {
    name: sys.modules.get(name) for name in _STUB_MODULE_NAMES
}


class FakeTime:
    def __init__(self, nanoseconds=0):
        self.nanoseconds = nanoseconds

    def __le__(self, other):
        return self.nanoseconds <= other.nanoseconds


class FakeTimer:
    instances = []

    def __init__(self, duration, callback, oneshot=False):
        self.duration = duration
        self.callback = callback
        self.oneshot = oneshot
        self.stopped = False
        self.instances.append(self)

    def shutdown(self):
        self.stopped = True


def _message_module(name, **messages):
    package = ModuleType(name)
    message_module = ModuleType(f'{name}.msg')
    for message_name, message_type in messages.items():
        setattr(message_module, message_name, message_type)
    package.msg = message_module
    sys.modules[name] = package
    sys.modules[f'{name}.msg'] = message_module


rospy = ModuleType('rospy')
rospy.Duration = lambda seconds: seconds
rospy.Timer = FakeTimer
rospy.Time = FakeTime
rospy.ServiceException = RuntimeError
rospy.is_shutdown = lambda: False
rospy.loginfo = lambda *args, **kwargs: None
rospy.logwarn = lambda *args, **kwargs: None
rospy.logerr = lambda *args, **kwargs: None
sys.modules['rospy'] = rospy

_message_module('geometry_msgs', PoseStamped=object)
_message_module('nav_msgs', Odometry=object, Path=object)
std_srvs = ModuleType('std_srvs')
std_srvs_srv = ModuleType('std_srvs.srv')
std_srvs_srv.SetBool = object
std_srvs_srv.Trigger = object
std_srvs_srv.TriggerResponse = object
std_srvs.srv = std_srvs_srv
sys.modules['std_srvs'] = std_srvs
sys.modules['std_srvs.srv'] = std_srvs_srv

MODULE_PATH = (
    Path(__file__).parents[1] / 'scripts' / 'ros1' / 'waypoint_follower.py'
)
SPEC = importlib.util.spec_from_file_location('waypoint_follower', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
WaypointFollower = MODULE.WaypointFollower

# The production module keeps references to the stubs it needs for these unit
# tests. Restore the interpreter's module registry so other ROS tests import
# their real message type support.
for name, original_module in _ORIGINAL_MODULES.items():
    if original_module is None:
        sys.modules.pop(name, None)
    else:
        sys.modules[name] = original_module


def _make_follower():
    follower = WaypointFollower.__new__(WaypointFollower)
    follower.running = True
    follower.restart_timer = None
    follower.restart_generation = 1
    follower.restart_attempts = 0
    follower.awaiting_restarted_path = True
    follower.restart_goal_stamp = FakeTime(100)
    follower.retry_delay = 2.0
    follower.max_retries = 0
    follower.planner_started = True
    follower.start_planner = lambda: SimpleNamespace(success=True, message='')
    return follower


def test_restart_is_retried_until_a_newer_path_arrives():
    FakeTimer.instances.clear()
    follower = _make_follower()

    follower._restart_planner(None, 1)

    assert follower.restart_attempts == 1
    assert follower.awaiting_restarted_path
    assert len(FakeTimer.instances) == 1
    assert FakeTimer.instances[0].duration == 2.0

    stale_path = SimpleNamespace(
        poses=[object()], header=SimpleNamespace(stamp=FakeTime(100))
    )
    follower._path_callback(stale_path)
    assert follower.awaiting_restarted_path
    assert not FakeTimer.instances[0].stopped

    fresh_path = SimpleNamespace(
        poses=[object()], header=SimpleNamespace(stamp=FakeTime(101))
    )
    follower._path_callback(fresh_path)
    assert not follower.awaiting_restarted_path
    assert FakeTimer.instances[0].stopped
    assert follower.restart_timer is None


def test_old_timer_generation_cannot_restart_a_new_goal():
    FakeTimer.instances.clear()
    follower = _make_follower()
    follower.restart_generation = 2

    follower._restart_planner(None, 1)

    assert follower.restart_attempts == 0
    assert FakeTimer.instances == []
