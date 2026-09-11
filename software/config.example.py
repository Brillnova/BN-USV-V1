"""Copy this file to config.py and set local ports, waypoints and calibration values."""

GPS_PORT = "/dev/ttyAMA0"
GPS_BAUD = 9600
ESP_PORT = "/dev/ttyACM0"
ESP_BAUD = 115200
MAG_ADDR = 0x1C

ARRIVAL_RADIUS = 5.0
BASE_WAYPOINTS = [
    {"lat": 0.0, "lon": 0.0},
]
REPEAT_COUNT = 1
WAYPOINTS = BASE_WAYPOINTS * REPEAT_COUNT

# Replace these placeholders after calibrating the magnetometer on your vessel.
MAG_OFFSET_X = 0.0
MAG_OFFSET_Y = 0.0
MAG_SCALE_X = 1.0
MAG_SCALE_Y = 1.0
MAG_AVG_SCALE = 1.0
