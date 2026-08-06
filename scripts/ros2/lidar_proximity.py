"""Point-cloud proximity checks used by the GBPlanner selector node."""

import numpy as np


class DurationThresholdFilter:
    """Change a Boolean state only after an observation remains stable."""

    def __init__(self, initial_value, duration_threshold):
        if duration_threshold <= 0.0:
            raise ValueError('duration_threshold must be positive')

        self.value = bool(initial_value)
        self.duration_threshold_ns = int(duration_threshold * 1e9)
        self.pending_value = None
        self.pending_since_ns = None

    def update(self, observed_value, now_ns):
        """Apply an observation made at ``now_ns`` and return the state."""
        observed_value = bool(observed_value)

        if observed_value == self.value:
            self.pending_value = None
            self.pending_since_ns = None
            return self.value

        if observed_value != self.pending_value:
            self.pending_value = observed_value
            self.pending_since_ns = now_ns
            return self.value

        # Restart the stability interval if ROS time jumps backwards, such as
        # when a simulation is reset.
        if now_ns < self.pending_since_ns:
            self.pending_since_ns = now_ns
            return self.value

        if now_ns - self.pending_since_ns >= self.duration_threshold_ns:
            self.value = observed_value
            self.pending_value = None
            self.pending_since_ns = None

        return self.value

    def set_immediately(self, value):
        """Set the state immediately and discard any pending transition."""
        self.value = bool(value)
        self.pending_value = None
        self.pending_since_ns = None
        return self.value


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
