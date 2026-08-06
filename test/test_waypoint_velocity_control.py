import importlib.util
import math
from pathlib import Path

import pytest


MODULE_PATH = (
    Path(__file__).parents[1] / 'scripts' / 'ros2' / 'waypoint_velocity_control.py'
)
SPEC = importlib.util.spec_from_file_location('waypoint_velocity_control', MODULE_PATH)
control = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(control)


def test_obstacle_speed_limit_clamps_and_interpolates():
    args = (1.0, 5.0, 0.2, 2.0)
    assert control.obstacle_speed_limit(0.5, *args) == pytest.approx(0.2)
    assert control.obstacle_speed_limit(3.0, *args) == pytest.approx(1.1)
    assert control.obstacle_speed_limit(8.0, *args) == pytest.approx(2.0)


def test_waypoint_limit_defaults_to_reaching_zero_at_the_waypoint():
    assert control.waypoint_speed_limit(0.0, 0.5, 2.0) == 0.0
    assert control.waypoint_speed_limit(1.0, 0.5, 2.0) == pytest.approx(1.0)
    assert control.waypoint_speed_limit(100.0, 0.5, 2.0) == 2.0


def test_waypoint_limit_reaches_arrival_speed_at_arrival_distance():
    args = (1.0, 4.0, 5.0, 1.0)
    assert control.waypoint_speed_limit(3.0, *args) == pytest.approx(1.0)
    assert control.waypoint_speed_limit(5.0, *args) == pytest.approx(1.0)
    assert control.waypoint_speed_limit(7.0, *args) == pytest.approx(math.sqrt(5.0))
    assert control.waypoint_speed_limit(100.0, *args) == 4.0


def test_waypoint_limit_rejects_arrival_speed_above_maximum():
    with pytest.raises(ValueError, match='waypoint_arrival_speed'):
        control.waypoint_speed_limit(5.0, 1.0, 2.0, 5.0, 3.0)


def test_slew_speed_limits_acceleration_and_deceleration():
    assert control.slew_speed(0.0, 2.0, 0.5, 0.1) == pytest.approx(0.05)
    assert control.slew_speed(1.0, 0.0, 0.5, 0.1) == pytest.approx(0.95)
    assert control.slew_speed(0.9, 1.0, 0.5, 1.0) == pytest.approx(1.0)


def test_shortest_angular_distance_wraps():
    error = control.shortest_angular_distance(math.radians(179), math.radians(-179))
    assert error == pytest.approx(math.radians(2))
