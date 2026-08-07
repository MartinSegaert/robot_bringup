"""Pure control helpers for the waypoint velocity commander."""

import math


def clamp(value, lower, upper):
    """Clamp ``value`` to the inclusive range [lower, upper]."""
    return max(lower, min(value, upper))


def distance_speed_limit(
    distance, min_distance, max_distance, min_speed, max_speed
):
    """Linearly map a distance to a speed within the configured bounds."""
    if max_distance <= min_distance:
        raise ValueError('max_distance must be greater than min_distance')
    if min_speed < 0.0 or max_speed < min_speed:
        raise ValueError('speeds must satisfy 0 <= min_speed <= max_speed')

    ratio = clamp(
        (distance - min_distance) / (max_distance - min_distance),
        0.0,
        1.0,
    )
    return min_speed + ratio * (max_speed - min_speed)


def obstacle_speed_limit(
    obstacle_distance, min_distance, max_distance, min_speed, max_speed
):
    """Linearly map obstacle clearance to an allowed speed."""
    return distance_speed_limit(
        obstacle_distance, min_distance, max_distance, min_speed, max_speed
    )


def waypoint_speed_limit(
    distance,
    max_acceleration,
    max_speed,
    waypoint_arrival_distance=0.0,
    waypoint_arrival_speed=0.0,
):
    """Return a speed that reaches the requested speed at the arrival radius."""
    if max_acceleration <= 0.0:
        raise ValueError('max_acceleration must be positive')
    if max_speed < 0.0:
        raise ValueError('max_speed must be non-negative')
    if waypoint_arrival_distance < 0.0:
        raise ValueError('waypoint_arrival_distance must be non-negative')
    if not 0.0 <= waypoint_arrival_speed <= max_speed:
        raise ValueError(
            'waypoint_arrival_speed must be between zero and max_speed'
        )
    braking_distance = max(distance - waypoint_arrival_distance, 0.0)
    return min(
        max_speed,
        math.sqrt(
            waypoint_arrival_speed**2
            + 2.0 * max_acceleration * braking_distance
        ),
    )


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
