from pathlib import Path
import sys

import numpy as np
import pytest


SCRIPT_DIR = Path(__file__).parents[1] / 'scripts' / 'ros2'
sys.path.insert(0, str(SCRIPT_DIR))

from lidar_proximity import (  # noqa: E402
    DurationThresholdFilter,
    has_point_within_distance,
)


POINT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4')])


def make_points(xyz):
    points = np.zeros(len(xyz), dtype=POINT_DTYPE)
    if xyz:
        points['x'], points['y'], points['z'] = np.asarray(xyz).T
    return points


def test_detects_a_point_on_or_inside_threshold():
    assert has_point_within_distance(
        make_points([(6.0, 0.0, 0.0), (3.0, 4.0, 0.0)]), 5.0
    )


def test_returns_false_when_no_valid_point_is_close():
    points = make_points(
        [(5.01, 0.0, 0.0), (np.nan, 0.0, 0.0), (np.inf, 0.0, 0.0)]
    )
    assert not has_point_within_distance(points, 5.0)
    assert not has_point_within_distance(make_points([]), 5.0)


def test_rejects_non_positive_threshold():
    with pytest.raises(ValueError, match='must be positive'):
        has_point_within_distance(make_points([(1.0, 0.0, 0.0)]), 0.0)


def test_duration_filter_requires_a_continuous_observation():
    state_filter = DurationThresholdFilter(True, duration_threshold=1.0)

    assert state_filter.update(False, 0) is True
    assert state_filter.update(False, 999_999_999) is True
    assert state_filter.update(True, 1_000_000_000) is True

    # Returning to the current state reset the pending false transition.
    assert state_filter.update(False, 1_100_000_000) is True
    assert state_filter.update(False, 2_100_000_000) is False


def test_duration_filter_debounces_both_directions():
    state_filter = DurationThresholdFilter(True, duration_threshold=0.5)

    assert state_filter.update(False, 0) is True
    assert state_filter.update(False, 500_000_000) is False
    assert state_filter.update(True, 600_000_000) is False
    assert state_filter.update(True, 1_100_000_000) is True


def test_duration_filter_can_be_set_immediately():
    state_filter = DurationThresholdFilter(False, duration_threshold=1.0)

    assert state_filter.update(True, 0) is False
    assert state_filter.set_immediately(True) is True
    assert state_filter.pending_value is None
    assert state_filter.pending_since_ns is None


def test_duration_filter_restarts_after_time_moves_backwards():
    state_filter = DurationThresholdFilter(True, duration_threshold=1.0)

    assert state_filter.update(False, 2_000_000_000) is True
    assert state_filter.update(False, 1_000_000_000) is True
    assert state_filter.update(False, 1_999_999_999) is True
    assert state_filter.update(False, 2_000_000_000) is False


def test_duration_filter_rejects_non_positive_duration():
    with pytest.raises(ValueError, match='must be positive'):
        DurationThresholdFilter(True, duration_threshold=0.0)
