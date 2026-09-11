# navigation.py
import math


def calculate_bearing(lat1, lon1, lat2, lon2):
    """Calculate bearing from current position to target position."""
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    diff_lon = math.radians(lon2 - lon1)

    x = math.sin(diff_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (
        math.sin(lat1) * math.cos(lat2) * math.cos(diff_lon)
    )

    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360


def calculate_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS coordinates in meters."""
    earth_radius = 6371000

    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = (
        math.sin(dphi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    )

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return earth_radius * c


def normalize_angle_error(target_angle, current_angle):
    """Normalize angle error to -180 ~ +180 degrees."""
    error = target_angle - current_angle

    while error > 180:
        error -= 360

    while error < -180:
        error += 360

    return error