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
    """Replace far non-returns with finite points along their beams.

    Gazebo's GPU lidar publishes an organized cloud in row-major order:
    columns sweep horizontally and rows sweep vertically. That organization
    lets us recover a non-return's beam direction without using its Cartesian
    coordinates. Gazebo represents returns beyond the maximum range with a
    positive infinite range and returns before the minimum range with a
    negative infinite range. Only the former indicates free space; near and
    otherwise ambiguous non-returns are removed.
    """
    if not {'x', 'y', 'z'}.issubset(points.dtype.names or ()):
        raise ValueError('Point cloud must contain x, y, and z fields')

    xyz = np.column_stack((points['x'], points['y'], points['z']))
    non_return_mask = ~np.isfinite(xyz).all(axis=1)
    if not np.any(non_return_mask):
        return points, 0

    if not allow_clear:
        return points[~non_return_mask], 0

    source_indices = np.asarray(source_indices)
    if source_indices.shape != non_return_mask.shape:
        raise ValueError('There must be one source index per point')
    if (
        width <= 0
        or height <= 0
        or np.any(source_indices < 0)
        or np.any(source_indices >= width * height)
    ):
        raise ValueError('Point cloud dimensions do not match its point data')

    non_return_indices = source_indices[non_return_mask]
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

    cos_elevation = np.cos(elevation)
    directions = np.column_stack(
        (
            cos_elevation * np.cos(azimuth),
            cos_elevation * np.sin(azimuth),
            np.sin(elevation),
        )
    )

    # Cartesian non-returns are signed infinities multiplied by the beam unit
    # vector. Inspect its largest component so zero-valued components (which
    # become NaN through inf * 0) cannot hide the sign of the original range.
    non_return_xyz = xyz[non_return_mask]
    dominant_axes = np.argmax(np.abs(directions), axis=1)
    dominant_values = non_return_xyz[
        np.arange(len(non_return_xyz)), dominant_axes
    ]
    dominant_directions = directions[
        np.arange(len(directions)), dominant_axes
    ]
    far_non_return_mask = np.isinf(dominant_values) & (
        np.signbit(dominant_values) == np.signbit(dominant_directions)
    )

    non_return_point_indices = np.flatnonzero(non_return_mask)
    far_point_indices = non_return_point_indices[far_non_return_mask]
    endpoint_distance = max_ray_length_m + clearing_ray_margin_m
    far_directions = directions[far_non_return_mask]

    points['x'][far_point_indices] = endpoint_distance * far_directions[:, 0]
    points['y'][far_point_indices] = endpoint_distance * far_directions[:, 1]
    points['z'][far_point_indices] = endpoint_distance * far_directions[:, 2]

    keep_mask = ~non_return_mask
    keep_mask[far_point_indices] = True

    return points[keep_mask], int(np.count_nonzero(far_non_return_mask))
