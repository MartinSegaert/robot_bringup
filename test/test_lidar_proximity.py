from pathlib import Path
import sys

import numpy as np
import pytest


SCRIPT_DIR = Path(__file__).parents[1] / 'scripts' / 'ros2'
sys.path.insert(0, str(SCRIPT_DIR))

from lidar_proximity import has_point_within_distance  # noqa: E402


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
