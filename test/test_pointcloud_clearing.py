from pathlib import Path
import sys

import numpy as np


SCRIPT_DIR = Path(__file__).parents[1] / 'scripts' / 'ros2'
sys.path.insert(0, str(SCRIPT_DIR))

from pointcloud_clearing import replace_non_returns  # noqa: E402


POINT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4')])


def make_points(depths, width, height, horizontal_angles, vertical_angles):
    points = np.zeros(width * height, dtype=POINT_DTYPE)
    directions = []
    for elevation in vertical_angles:
        for azimuth in horizontal_angles:
            directions.append(
                (
                    np.cos(elevation) * np.cos(azimuth),
                    np.cos(elevation) * np.sin(azimuth),
                    np.sin(elevation),
                )
            )
    directions = np.asarray(directions)
    with np.errstate(invalid='ignore'):
        xyz = np.asarray(depths)[:, None] * directions
    points['x'], points['y'], points['z'] = xyz.T
    return points, directions


def test_only_far_non_returns_become_clearing_rays():
    width = 4
    height = 2
    horizontal_angles = np.linspace(-2.4, 1.2, width)
    vertical_angles = np.linspace(-0.2, 0.7, height)
    depths = [2.0, np.inf, -np.inf, np.nan, np.inf, 3.0, -np.inf, np.inf]
    points, directions = make_points(
        depths, width, height, horizontal_angles, vertical_angles
    )

    result, replaced_count = replace_non_returns(
        points=points,
        source_indices=np.arange(width * height),
        width=width,
        height=height,
        allow_clear=True,
        max_ray_length_m=20.0,
        clearing_ray_margin_m=0.1,
        horizontal_min_angle=horizontal_angles[0],
        horizontal_max_angle=horizontal_angles[-1],
        vertical_min_angle=vertical_angles[0],
        vertical_max_angle=vertical_angles[-1],
    )

    assert replaced_count == 3
    assert len(result) == 5
    result_xyz = np.column_stack((result['x'], result['y'], result['z']))
    expected_xyz = np.vstack(
        (
            2.0 * directions[0],
            20.1 * directions[1],
            20.1 * directions[4],
            3.0 * directions[5],
            20.1 * directions[7],
        )
    )
    np.testing.assert_allclose(result_xyz, expected_xyz, rtol=1e-6)
    assert np.isfinite(result_xyz).all()


def test_disabling_clearing_removes_every_non_return():
    width = 2
    height = 1
    horizontal_angles = np.array([-0.4, 0.4])
    vertical_angles = np.array([0.2])
    points, directions = make_points(
        [4.0, np.inf], width, height, horizontal_angles, vertical_angles
    )

    result, replaced_count = replace_non_returns(
        points=points,
        source_indices=np.arange(width),
        width=width,
        height=height,
        allow_clear=False,
        max_ray_length_m=20.0,
        clearing_ray_margin_m=0.1,
        horizontal_min_angle=horizontal_angles[0],
        horizontal_max_angle=horizontal_angles[-1],
        vertical_min_angle=vertical_angles[0],
        vertical_max_angle=vertical_angles[0],
    )

    assert replaced_count == 0
    assert len(result) == 1
    np.testing.assert_allclose(
        np.array([result['x'][0], result['y'][0], result['z'][0]]),
        4.0 * directions[0],
        rtol=1e-6,
    )
