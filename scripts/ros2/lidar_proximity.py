"""Point-cloud proximity checks used by the GBPlanner selector node."""

import numpy as np


def has_point_within_distance(points, distance_threshold):
    """Return whether any finite XYZ point is within the given radius."""
    if distance_threshold <= 0.0:
        raise ValueError('distance_threshold must be positive')
    if not {'x', 'y', 'z'}.issubset(points.dtype.names or ()):
        raise ValueError('Point cloud must contain x, y, and z fields')

    xyz = np.column_stack((points['x'], points['y'], points['z']))
    finite_xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(finite_xyz) == 0:
        return False

    squared_distances = np.einsum('ij,ij->i', finite_xyz, finite_xyz)
    return bool(np.any(squared_distances <= distance_threshold**2))
