# V1 reference software

This folder contains the Raspberry Pi-side reference implementation for V1 navigation, sensor input, propulsion commands and runtime logging. It is intended for inspection and adaptation, not as a turnkey deployment package.

## Before running

Copy `config.example.py` to `config.py`, then set your local serial ports, waypoint coordinates and magnetometer calibration values. `config.py` is deliberately ignored by Git so local settings and test locations are not published.

Install the minimal Python dependencies with:

```bash
python3 -m pip install -r requirements.txt
```

`main_v4.py` is the main application entry point. It expects the configured sensors and an ESP32 connection to be available.

## Files

| File | Purpose |
| --- | --- |
| `main_v4.py` | Runs the V1 mission loop: reads sensors, calculates navigation and control output, sends commands to the ESP32, and records state. |
| `config.example.py` | Template for local ports, waypoints, arrival radius and magnetometer calibration. Copy it to `config.py` before use. |
| `gps.py` | Reads and parses GPS/NMEA position, speed and course information. |
| `mag.py` | Reads the LIS3MDL magnetometer and applies heading calibration values. |
| `mag_calibration.py` | Updates magnetometer heading calibration from GPS-course comparisons when suitable data is available. |
| `imu.py` | Reads IMU acceleration and angular-rate data. |
| `battery.py` | Reads the INA226 battery monitor and estimates voltage, current and state of charge. |
| `navigation.py` | Calculates distance, bearing and normalized heading error to the current waypoint. |
| `controller.py` | Converts distance and heading error into throttle and steering commands. |
| `esp_link.py` | Exchanges JSON-line control commands and telemetry with the ESP32 over serial. |
| `logger.py` | Writes navigation, sensor, battery and control state to CSV and JSONL logs. |
| `requirements.txt` | Minimal Python dependency list for the reference software. |

## Operational notes

The reference controller and all safety thresholds need review and testing for the vessel, sensors and operating conditions in use. Test with propulsion safely restrained or disconnected before water operation. Raw logs are not included in this public repository.
