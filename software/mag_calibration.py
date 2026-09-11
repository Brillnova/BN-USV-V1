# mag_calibration.py


def normalize_angle(angle):
    """Normalize angle to 0 ~ 360 degrees."""
    angle = angle % 360.0
    if angle < 0:
        angle += 360.0
    return angle


def normalize_error(error):
    """Normalize angle error to -180 ~ +180 degrees."""
    while error > 180:
        error -= 360

    while error < -180:
        error += 360

    return error


class MagAutoCalibrator:
    def __init__(
        self,
        min_speed_mps=0.7,
        max_error_deg=45.0,
        max_steering_for_update=0.20,
        alpha=0.02,
        initial_offset_deg=0.0,
    ):
        self.min_speed_mps = min_speed_mps
        self.max_error_deg = max_error_deg
        self.max_steering_for_update = max_steering_for_update
        self.alpha = alpha
        self.offset_deg = initial_offset_deg
        self.valid_update_count = 0

    def apply(self, mag_heading):
        """Apply learned heading offset to raw mag heading."""
        return normalize_angle(mag_heading + self.offset_deg)

    def update(self, mag_heading, gps_course, speed_mps, steering):
        """
        Update heading offset using GPS course over ground.

        Offset is updated only when:
        - GPS course is available
        - Vehicle speed is high enough
        - Steering command is small enough
        - GPS/MAG difference is within a reasonable range
        """
        if gps_course is None:
            return self.offset_deg, False

        if speed_mps < self.min_speed_mps:
            return self.offset_deg, False

        if abs(steering) > self.max_steering_for_update:
            return self.offset_deg, False

        corrected_heading = self.apply(mag_heading)
        error = normalize_error(gps_course - corrected_heading)

        if abs(error) > self.max_error_deg:
            return self.offset_deg, False

        self.offset_deg += self.alpha * error
        self.offset_deg = normalize_error(self.offset_deg)

        self.valid_update_count += 1

        return self.offset_deg, True