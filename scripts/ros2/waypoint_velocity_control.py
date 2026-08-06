"""Pure control helpers for the waypoint velocity commander."""

import math


def clamp(value, lower, upper):
    """Clamp ``value`` to the inclusive range [lower, upper]."""
    return max(lower, min(value, upper))


def obstacle_speed_limit(
    obstacle_distance, min_distance, max_distance, min_speed, max_speed
):
    """Linearly map obstacle clearance to an allowed speed."""
    if max_distance <= min_distance:
        raise ValueError('max_distance must be greater than min_distance')
    if min_speed < 0.0 or max_speed < min_speed:
        raise ValueError('speeds must satisfy 0 <= min_speed <= max_speed')

    ratio = clamp(
        (obstacle_distance - min_distance) / (max_distance - min_distance),
        0.0,
        1.0,
    )
    return min_speed + ratio * (max_speed - min_speed)


def waypoint_speed_limit(
    distance, max_acceleration, max_speed, stopping_tolerance=0.0
):
    """Return a speed that reaches zero at ``stopping_tolerance``."""
    if max_acceleration <= 0.0:
        raise ValueError('max_acceleration must be positive')
    if max_speed < 0.0:
        raise ValueError('max_speed must be non-negative')
    if stopping_tolerance < 0.0:
        raise ValueError('stopping_tolerance must be non-negative')
    braking_distance = max(distance - stopping_tolerance, 0.0)
    return min(max_speed, math.sqrt(2.0 * max_acceleration * braking_distance))


def slew_speed(current_speed, target_speed, max_acceleration, dt):
    """Move speed toward its target without exceeding the acceleration limit."""
    if max_acceleration <= 0.0:
        raise ValueError('max_acceleration must be positive')
    if dt <= 0.0:
        return current_speed
    max_change = max_acceleration * dt
    return current_speed + clamp(target_speed - current_speed, -max_change, max_change)


def shortest_angular_distance(current, target):
    """Return the signed shortest angle from current to target."""
    return math.atan2(math.sin(target - current), math.cos(target - current))
