# controller.py


def clamp(value, min_value, max_value):
    """Clamp value between min and max."""
    return max(min_value, min(max_value, value))


def calculate_control(heading_error, distance, arrival_radius):
    """Convert heading error and distance to throttle and steering."""

    steering = heading_error / 45.0
    steering = clamp(steering, -1.0, 1.0)

    if distance < arrival_radius:
        throttle = 0.0
        steering = 0.0
    elif abs(heading_error) > 90:
        throttle = 0.10
    else:
        throttle = 0.25

    return throttle, steering