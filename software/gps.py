# gps.py
import serial
import time


class GPS:
    def __init__(self, port, baudrate):
        self.serial = serial.Serial(port, baudrate, timeout=0)  # Non-blocking
        self.last_lat = None
        self.last_lon = None
        self.last_speed = 0.0
        self.last_course = None
        self.last_hdop = None
        self.last_satellites = None
        self.last_fix_quality = None

    def convert_to_decimal(self, raw, direction):
        """Convert NMEA coordinate format to decimal degrees."""
        raw = float(raw)
        degrees = int(raw / 100)
        minutes = raw - (degrees * 100)
        decimal = degrees + minutes / 60

        if direction in ["S", "W"]:
            decimal = -decimal

        return decimal

    def parse_rmc(self, line):
        """Parse RMC sentence and return lat, lon, speed, course."""
        parts = line.split(",")

        if len(parts) < 9:
            return None

        # RMC status: A = valid, V = invalid
        if parts[2] != "A":
            return None

        if not parts[3] or not parts[4] or not parts[5] or not parts[6]:
            return None

        lat = self.convert_to_decimal(parts[3], parts[4])
        lon = self.convert_to_decimal(parts[5], parts[6])

        speed_knots = float(parts[7]) if parts[7] else 0.0
        speed_mps = speed_knots * 0.514444

        course_deg = float(parts[8]) if parts[8] else None

        return lat, lon, speed_mps, course_deg

    def parse_gga(self, line):
        """Parse GGA sentence and update GPS quality fields."""
        parts = line.split(",")

        if len(parts) < 9:
            return None

        try:
            fix_quality = int(parts[6]) if parts[6] else 0
            satellites = int(parts[7]) if parts[7] else None
            hdop = float(parts[8]) if parts[8] else None
        except ValueError:
            return None

        self.last_fix_quality = fix_quality
        self.last_satellites = satellites
        self.last_hdop = hdop

        return fix_quality, satellites, hdop

    def poll_position(self):
        """
        Non-blocking GPS poll. Read available NMEA sentences without waiting.

        Returns:
            lat, lon, speed_mps, course_deg, hdop, satellites, fix_quality

        If GPS is updated, returns new values. Otherwise returns last valid values.
        Returns None for lat/lon if never updated, but preserves quality fields.
        """
        # Read all available data
        while self.serial.in_waiting > 0:
            try:
                line = self.serial.readline().decode("ascii", errors="replace").strip()
            except Exception:
                break

            if not line:
                continue

            if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                self.parse_gga(line)
                continue

            if line.startswith("$GNRMC") or line.startswith("$GPRMC"):
                data = self.parse_rmc(line)
                if data is not None:
                    self.last_lat, self.last_lon, self.last_speed, self.last_course = data

        # Always return current state (last valid values)
        return (
            self.last_lat,
            self.last_lon,
            self.last_speed,
            self.last_course,
            self.last_hdop,
            self.last_satellites,
            self.last_fix_quality,
        )

    def read_position(self, timeout_sec=1.0):
        """
        Blocking read until a valid RMC sentence is received or timeout is reached.

        Returns:
            lat, lon, speed_mps, course_deg, hdop, satellites, fix_quality
        """
        # Temporarily set timeout for blocking read
        self.serial.timeout = timeout_sec
        start_time = time.time()

        while True:
            if time.time() - start_time >= timeout_sec:
                self.serial.timeout = 0  # Reset to non-blocking
                return (
                    self.last_lat,
                    self.last_lon,
                    self.last_speed,
                    self.last_course,
                    self.last_hdop,
                    self.last_satellites,
                    self.last_fix_quality,
                )

            try:
                line = self.serial.readline().decode("ascii", errors="replace").strip()
            except Exception:
                continue

            if not line:
                continue

            if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                self.parse_gga(line)
                continue

            if line.startswith("$GNRMC") or line.startswith("$GPRMC"):
                data = self.parse_rmc(line)
                if data is not None:
                    self.last_lat, self.last_lon, self.last_speed, self.last_course = data
                    self.serial.timeout = 0  # Reset to non-blocking
                    return (
                        self.last_lat,
                        self.last_lon,
                        self.last_speed,
                        self.last_course,
                        self.last_hdop,
                        self.last_satellites,
                        self.last_fix_quality,
                    )
