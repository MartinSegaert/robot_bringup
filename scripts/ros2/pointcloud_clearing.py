"""Utilities for turning organized lidar non-returns into clearing rays."""

import numpy as np


def replace_non_returns(
    points,
    source_indices,
    width,
    height,
    allow_clear,
    max_ray_length_m,
    clearing_ray_margin_m,
    horizontal_min_angle,
    horizontal_max_angle,
    vertical_min_angle,
    vertical_max_angle,
):
    """Replace non-finite XYZ samples with finite points along their beams.

    Gazebo's GPU lidar publishes an organized cloud in row-major order:
    columns sweep horizontally and rows sweep vertically. That organization
    lets us recover a non-return's beam direction without using its inf/NaN
    Cartesian coordinates.
    """
    if not {'x', 'y', 'z'}.issubset(points.dtype.names or ()):
        raise ValueError('Point cloud must contain x, y, and z fields')

    xyz = np.column_stack((points['x'], points['y'], points['z']))
    non_return_mask = ~np.isfinite(xyz).all(axis=1)
    if not np.any(non_return_mask):
        return points, 0

    if not allow_clear:
        return points[~non_return_mask], 0

    if width <= 0 or height <= 0 or width * height <= np.max(source_indices):
        raise ValueError('Point cloud dimensions do not match its point data')

    non_return_indices = np.asarray(source_indices)[non_return_mask]
    rows = non_return_indices // width
    columns = non_return_indices % width

    if width == 1:
        azimuth = np.full(columns.shape, horizontal_min_angle, dtype=np.float64)
    else:
        azimuth = horizontal_min_angle + columns * (
            (horizontal_max_angle - horizontal_min_angle) / (width - 1)
        )

    if height == 1:
        elevation = np.full(rows.shape, vertical_min_angle, dtype=np.float64)
    else:
        elevation = vertical_min_angle + rows * (
            (vertical_max_angle - vertical_min_angle) / (height - 1)
        )

    endpoint_distance = max_ray_length_m + clearing_ray_margin_m
    cos_elevation = np.cos(elevation)

    points['x'][non_return_mask] = (
        endpoint_distance * cos_elevation * np.cos(azimuth)
    )
    points['y'][non_return_mask] = (
        endpoint_distance * cos_elevation * np.sin(azimuth)
    )
    points['z'][non_return_mask] = endpoint_distance * np.sin(elevation)

    return points, int(np.count_nonzero(non_return_mask))
