# main.py
import json
import os
import time
from contextlib import suppress

from config import (
    GPS_PORT,
    GPS_BAUD,
    ESP_PORT,
    ESP_BAUD,
    MAG_ADDR,
    WAYPOINTS,
    ARRIVAL_RADIUS,
    MAG_OFFSET_X,
    MAG_OFFSET_Y,
    MAG_SCALE_X,
    MAG_SCALE_Y,
    MAG_AVG_SCALE,
)

from gps import GPS
from mag import Magnetometer
from mag_calibration import MagAutoCalibrator
from navigation import (
    calculate_bearing,
    calculate_distance,
    normalize_angle_error,
)
from controller import calculate_control
from esp_link import EspLink
from logger import USVLogger
from imu import IMU
from battery import BatteryMonitor


PRINT_INTERVAL_SEC = 1.0
ESP_CMD_SEND_INTERVAL_SEC = 0.1  # 10Hz command rate to prevent ESP watchdog timeout
LOOP_SLEEP_SEC = 0.01  # Allow fast loop with polling, actual rate controlled by sleep below


def empty_esp_status():
    """Return an empty ESP telemetry status record for CSV-safe logging."""
    return {
        "esp_mode": "",
        "esp_rc_mode": "",
        "esp_rc_ok": "",
        "esp_pi_fresh": "",
        "esp_manual_latch": "",
        "esp_ch1_us": "",
        "esp_ch2_us": "",
        "esp_ch4_us": "",
        "esp_cmd_thr_us": "",
        "esp_cmd_str_us": "",
        "esp_left_pwm": "",
        "esp_right_pwm": "",
    }


def parse_esp_telemetry_status(responses, previous_status=None):
    """
    Parse the latest telemetry_status message from ESP protocol lines.

    If responses contain only ack/error/debug lines and no telemetry_status,
    keep the previous ESP status instead of overwriting it with blank values.
    """
    status = dict(previous_status or empty_esp_status())

    for response in responses:
        try:
            msg = json.loads(response)
        except (TypeError, json.JSONDecodeError):
            continue

        if msg.get("type") != "telemetry_status":
            continue

        data = msg.get("data", {})
        status["esp_mode"] = data.get("mode", status.get("esp_mode", ""))
        status["esp_rc_mode"] = data.get("rc_mode", status.get("esp_rc_mode", ""))
        status["esp_rc_ok"] = data.get("rc_ok", status.get("esp_rc_ok", ""))
        status["esp_pi_fresh"] = data.get("pi_fresh", status.get("esp_pi_fresh", ""))
        status["esp_manual_latch"] = data.get(
            "manual_latch", status.get("esp_manual_latch", "")
        )
        status["esp_ch1_us"] = data.get("ch1_us", status.get("esp_ch1_us", ""))
        status["esp_ch2_us"] = data.get("ch2_us", status.get("esp_ch2_us", ""))
        status["esp_ch4_us"] = data.get("ch4_us", status.get("esp_ch4_us", ""))
        status["esp_cmd_thr_us"] = data.get(
            "cmd_thr_us", status.get("esp_cmd_thr_us", "")
        )
        status["esp_cmd_str_us"] = data.get(
            "cmd_str_us", status.get("esp_cmd_str_us", "")
        )
        status["esp_left_pwm"] = data.get("left_pwm", status.get("esp_left_pwm", ""))
        status["esp_right_pwm"] = data.get(
            "right_pwm", status.get("esp_right_pwm", "")
        )

    return status


def get_cpu_temperature():
    """Read Raspberry Pi CPU temperature in degrees Celsius, if available."""
    for path in (
        "/sys/class/thermal/thermal_zone0/temp",
        "/sys/class/thermal/thermal_zone1/temp",
    ):
        if not os.path.exists(path):
            continue

        try:
            with open(path, "r", encoding="utf-8") as temp_file:
                raw = temp_file.read().strip()
            return float(raw) / 1000.0
        except (OSError, ValueError):
            return None

    return None


def print_runtime_status(
    *,
    mission_state,
    waypoint_index,
    waypoint_total,
    distance,
    heading_error,
    speed,
    throttle,
    steering,
    battery_v,
    battery_a,
    battery_coulomb_soc,
    esp_status=None,
    pi_loop_dt=None,
    esp_response_count=0,
    gps_hdop=None,
    gps_satellites=None,
    gps_fix_quality=None,
    cpu_temp_c=None,
):
    """Print compact runtime status for operator monitoring."""
    print("\n-----------------------------")
    print(f"State: {mission_state}")
    print(f"Waypoint: {waypoint_index}/{waypoint_total}")
    print(f"Distance: {distance:.1f} m")
    print(f"Heading Error: {heading_error:.1f} deg")
    print(f"Speed: {speed:.2f} m/s")
    print(
        "GPS: "
        f"HDOP={gps_hdop if gps_hdop is not None else ''}, "
        f"SAT={gps_satellites if gps_satellites is not None else ''}, "
        f"FIX={gps_fix_quality if gps_fix_quality is not None else ''}"
    )
    print(f"Command: throttle={throttle:.2f}, steering={steering:.2f}")
    if cpu_temp_c is not None:
        print(f"CPU temp: {cpu_temp_c:.1f} C")
    print(
        f"Battery: {battery_v:.2f} V, "
        f"{battery_a:.2f} A, "
        f"SOC={battery_coulomb_soc:.1f}%"
    )

    if esp_status:
        print(
            "ESP: "
            f"mode={esp_status.get('esp_mode', '')}, "
            f"rc_mode={esp_status.get('esp_rc_mode', '')}, "
            f"rc_ok={esp_status.get('esp_rc_ok', '')}, "
            f"pi_fresh={esp_status.get('esp_pi_fresh', '')}, "
            f"L={esp_status.get('esp_left_pwm', '')}, "
            f"R={esp_status.get('esp_right_pwm', '')}"
        )

    if pi_loop_dt is not None:
        print(f"Pi loop: {pi_loop_dt:.3f} s")

    print(f"ESP responses logged: {esp_response_count}")


def has_gps_fix(lat, lon, gps_fix_quality):
    return lat is not None and lon is not None and gps_fix_quality is not None and gps_fix_quality > 0


def log_nav_row(
    logger,
    *,
    waypoint_index,
    waypoint_total,
    target_lat,
    target_lon,
    lat,
    lon,
    raw_heading,
    heading,
    gps_course,
    gps_hdop,
    gps_satellites,
    gps_fix_quality,
    offset_deg,
    cal_updated,
    bearing,
    heading_error,
    distance,
    speed,
    throttle,
    steering,
    ax,
    ay,
    az,
    gx,
    gy,
    gz,
    battery_v,
    battery_a,
    battery_voltage_soc,
    battery_coulomb_soc,
    battery_used_mah,
    battery_remaining_mah,
    battery_idle_corr,
    cpu_temp_c,
    mission_complete,
    pi_loop_dt,
    esp_status,
):
    """Write one navigation record to CSV."""
    row = {
        "time": time.time(),
        "waypoint_index": waypoint_index,
        "waypoint_total": waypoint_total,
        "target_lat": target_lat,
        "target_lon": target_lon,
        "current_lat": lat,
        "current_lon": lon,
        "raw_heading": raw_heading,
        "corrected_heading": heading,
        "gps_course": gps_course if gps_course is not None else "",
        "gps_hdop": gps_hdop if gps_hdop is not None else "",
        "gps_satellites": gps_satellites if gps_satellites is not None else "",
        "gps_fix_quality": gps_fix_quality if gps_fix_quality is not None else "",
        "mag_offset": offset_deg,
        "cal_updated": cal_updated,
        "bearing": bearing,
        "heading_error": heading_error,
        "distance": distance,
        "speed": speed,
        "throttle": throttle,
        "steering": steering,
        "ax": ax,
        "ay": ay,
        "az": az,
        "gx": gx,
        "gy": gy,
        "gz": gz,
        "battery_v": battery_v,
        "battery_a": battery_a,
        "battery_voltage_soc": battery_voltage_soc,
        "battery_coulomb_soc": battery_coulomb_soc,
        "battery_used_mah": battery_used_mah,
        "battery_remaining_mah": battery_remaining_mah,
        "battery_idle_corr": battery_idle_corr,
        "cpu_temp_c": cpu_temp_c,
        "mission_complete": mission_complete,
        "pi_loop_dt": pi_loop_dt,
    }
    row.update(esp_status or empty_esp_status())
    logger.log_nav(row)


def log_esp_responses(logger, responses):
    """Write raw ESP protocol responses to JSONL."""
    for response in responses:
        logger.log_esp_line(response)


def main():
    logger = USVLogger()
    gps = GPS(GPS_PORT, GPS_BAUD)
    mag = Magnetometer(
        address=MAG_ADDR,
        offset_x=MAG_OFFSET_X,
        offset_y=MAG_OFFSET_Y,
        scale_x=MAG_SCALE_X,
        scale_y=MAG_SCALE_Y,
        avg_scale=MAG_AVG_SCALE,
    )
    imu = IMU()

    mag.init()
    imu.init()
    battery = BatteryMonitor(
        capacity_mah=10000.0
    )

    mag_cal = MagAutoCalibrator(
        min_speed_mps=0.7,
        max_error_deg=45.0,
        max_steering_for_update=0.20,
        alpha=0.02,
        initial_offset_deg=0.0,
    )

    esp = EspLink(ESP_PORT, ESP_BAUD)

    current_wp_index = 0
    mission_complete = False
    last_print_time = 0.0
    last_esp_cmd_send_time = 0.0
    loop_target_dt = 0.1  # Target loop time: 10Hz
    esp_status = empty_esp_status()  # Cache ESP status, update only when new responses arrive

    print("USV multi-waypoint navigation started")
    print(f"Total waypoints: {len(WAYPOINTS)}")
    print("Control loop: 10Hz | GPS poll: non-blocking | ESP command: 10Hz")
    print("ESP status caching enabled - values updated only when new responses arrive")

    try:
        while True:
            loop_start = time.time()

            # Non-blocking GPS poll - updates position if new NMEA data available
            (
                lat,
                lon,
                speed,
                gps_course,
                gps_hdop,
                gps_satellites,
                gps_fix_quality,
            ) = gps.poll_position()

            raw_heading = mag.get_heading()
            heading = mag_cal.apply(raw_heading)

            ax, ay, az = imu.read_accel()
            gx, gy, gz = imu.read_gyro()

            battery_data = battery.read()

            battery_v = battery_data["voltage"]
            battery_a = battery_data["current"]
            battery_voltage_soc = battery_data["voltage_soc"]
            battery_coulomb_soc = battery_data["coulomb_soc"]
            battery_used_mah = battery_data["used_mah"]
            battery_remaining_mah = battery_data["remaining_mah"]
            battery_idle_corr = battery_data["idle_correction_applied"]
            cpu_temp_c = get_cpu_temperature()

            if mission_complete:
                throttle = 0.0
                steering = 0.0
                bearing = 0.0
                heading_error = 0.0
                distance = 0.0
                offset_deg = mag_cal.offset_deg
                cal_updated = False
                target_lat = lat if lat is not None else 0.0
                target_lon = lon if lon is not None else 0.0
                display_wp_index = len(WAYPOINTS)
                mission_state = "COMPLETE"

                now = time.time()
                # Send ESP command at 10Hz rate
                if now - last_esp_cmd_send_time >= ESP_CMD_SEND_INTERVAL_SEC:
                    esp.send_manual_command(throttle, steering)
                    last_esp_cmd_send_time = now

                responses = esp.read_available_lines()
                log_esp_responses(logger, responses)
                # Update ESP status from telemetry_status only; ack-only responses keep previous value
                if responses:
                    esp_status = parse_esp_telemetry_status(responses, esp_status)
                pi_loop_dt = time.time() - loop_start

                log_nav_row(
                    logger,
                    waypoint_index=display_wp_index,
                    waypoint_total=len(WAYPOINTS),
                    target_lat=target_lat,
                    target_lon=target_lon,
                    lat=lat if lat is not None else 0.0,
                    lon=lon if lon is not None else 0.0,
                    raw_heading=raw_heading,
                    heading=heading,
                    gps_course=gps_course,
                    gps_hdop=gps_hdop,
                    gps_satellites=gps_satellites,
                    gps_fix_quality=gps_fix_quality if gps_fix_quality else 0,
                    offset_deg=offset_deg,
                    cal_updated=cal_updated,
                    bearing=bearing,
                    heading_error=heading_error,
                    distance=distance,
                    speed=speed,
                    throttle=throttle,
                    steering=steering,
                    ax=ax,
                    ay=ay,
                    az=az,
                    gx=gx,
                    gy=gy,
                    gz=gz,
                    battery_v=battery_v,
                    battery_a=battery_a,
                    battery_voltage_soc=battery_voltage_soc,
                    battery_coulomb_soc=battery_coulomb_soc,
                    battery_used_mah=battery_used_mah,
                    battery_remaining_mah=battery_remaining_mah,
                    battery_idle_corr=battery_idle_corr,
                    cpu_temp_c=cpu_temp_c,
                    mission_complete=mission_complete,
                    pi_loop_dt=pi_loop_dt,
                    esp_status=esp_status,
                )

                if now - last_print_time >= PRINT_INTERVAL_SEC:
                    print_runtime_status(
                        mission_state=mission_state,
                        waypoint_index=display_wp_index,
                        waypoint_total=len(WAYPOINTS),
                        distance=distance,
                        heading_error=heading_error,
                        speed=speed,
                        gps_hdop=gps_hdop,
                        gps_satellites=gps_satellites,
                        gps_fix_quality=gps_fix_quality if gps_fix_quality else 0,
                        throttle=throttle,
                        steering=steering,
                        battery_v=battery_v,
                        battery_a=battery_a,
                        battery_coulomb_soc=battery_coulomb_soc,
                        esp_status=esp_status,
                        pi_loop_dt=pi_loop_dt,
                        esp_response_count=len(responses),
                        cpu_temp_c=cpu_temp_c,
                    )
                    last_print_time = now

                # Sleep to maintain 10Hz loop rate
                elapsed = time.time() - loop_start
                if elapsed < loop_target_dt:
                    time.sleep(loop_target_dt - elapsed)
                continue

            target = WAYPOINTS[current_wp_index]
            target_lat = target["lat"]
            target_lon = target["lon"]

            gps_ok = has_gps_fix(lat, lon, gps_fix_quality)

            if not gps_ok:
                mission_state = "NO GPS"
                throttle = 0.0
                steering = 0.0
                bearing = 0.0
                heading_error = 0.0
                distance = 0.0
                offset_deg = mag_cal.offset_deg
                cal_updated = False

                now = time.time()
                # Send ESP command at 10Hz rate
                if now - last_esp_cmd_send_time >= ESP_CMD_SEND_INTERVAL_SEC:
                    esp.send_manual_command(throttle, steering)
                    last_esp_cmd_send_time = now

                responses = esp.read_available_lines()
                log_esp_responses(logger, responses)
                # Update ESP status from telemetry_status only; ack-only responses keep previous value
                if responses:
                    esp_status = parse_esp_telemetry_status(responses, esp_status)
                display_wp_index = min(current_wp_index + 1, len(WAYPOINTS))
                pi_loop_dt = time.time() - loop_start

                log_nav_row(
                    logger,
                    waypoint_index=display_wp_index,
                    waypoint_total=len(WAYPOINTS),
                    target_lat=target_lat,
                    target_lon=target_lon,
                    lat=lat if lat is not None else 0.0,
                    lon=lon if lon is not None else 0.0,
                    raw_heading=raw_heading,
                    heading=heading,
                    gps_course=gps_course,
                    gps_hdop=gps_hdop,
                    gps_satellites=gps_satellites,
                    gps_fix_quality=0,
                    offset_deg=offset_deg,
                    cal_updated=cal_updated,
                    bearing=bearing,
                    heading_error=heading_error,
                    distance=distance,
                    speed=speed,
                    throttle=throttle,
                    steering=steering,
                    ax=ax,
                    ay=ay,
                    az=az,
                    gx=gx,
                    gy=gy,
                    gz=gz,
                    battery_v=battery_v,
                    battery_a=battery_a,
                    battery_voltage_soc=battery_voltage_soc,
                    battery_coulomb_soc=battery_coulomb_soc,
                    battery_used_mah=battery_used_mah,
                    battery_remaining_mah=battery_remaining_mah,
                    battery_idle_corr=battery_idle_corr,
                    cpu_temp_c=cpu_temp_c,
                    mission_complete=mission_complete,
                    pi_loop_dt=pi_loop_dt,
                    esp_status=esp_status,
                )

                if now - last_print_time >= PRINT_INTERVAL_SEC:
                    print("GPS lost. Autonomy paused, waiting for GPS fix. Maintaining 10Hz heartbeat...")
                    print_runtime_status(
                        mission_state=mission_state,
                        waypoint_index=display_wp_index,
                        waypoint_total=len(WAYPOINTS),
                        distance=distance,
                        heading_error=heading_error,
                        speed=speed,
                        gps_hdop=gps_hdop,
                        gps_satellites=gps_satellites,
                        gps_fix_quality=gps_fix_quality if gps_fix_quality else 0,
                        throttle=throttle,
                        steering=steering,
                        battery_v=battery_v,
                        battery_a=battery_a,
                        battery_coulomb_soc=battery_coulomb_soc,
                        esp_status=esp_status,
                        pi_loop_dt=pi_loop_dt,
                        esp_response_count=len(responses),
                        cpu_temp_c=cpu_temp_c,
                    )
                    last_print_time = now

                # Sleep to maintain 10Hz loop rate
                elapsed = time.time() - loop_start
                if elapsed < loop_target_dt:
                    time.sleep(loop_target_dt - elapsed)
                continue

            bearing = calculate_bearing(lat, lon, target_lat, target_lon)
            distance = calculate_distance(lat, lon, target_lat, target_lon)
            heading_error = normalize_angle_error(bearing, heading)

            throttle, steering = calculate_control(
                heading_error,
                distance,
                ARRIVAL_RADIUS,
            )

            offset_deg, cal_updated = mag_cal.update(
                mag_heading=raw_heading,
                gps_course=gps_course,
                speed_mps=speed,
                steering=steering,
            )

            heading = mag_cal.apply(raw_heading)
            heading_error = normalize_angle_error(bearing, heading)

            throttle, steering = calculate_control(
                heading_error,
                distance,
                ARRIVAL_RADIUS,
            )

            if distance < ARRIVAL_RADIUS:
                current_wp_index += 1

                throttle = 0.0
                steering = 0.0

                if current_wp_index >= len(WAYPOINTS):
                    mission_complete = True
                    print(">>> ALL WAYPOINTS COMPLETED <<<")
                else:
                    print(f">>> SWITCHING TO WAYPOINT {current_wp_index + 1} <<<")

            now = time.time()
            # Send ESP command at 10Hz rate to maintain pi_fresh heartbeat
            if now - last_esp_cmd_send_time >= ESP_CMD_SEND_INTERVAL_SEC:
                esp.send_manual_command(throttle, steering)
                last_esp_cmd_send_time = now

            responses = esp.read_available_lines()
            log_esp_responses(logger, responses)
            # Update ESP status from telemetry_status only; ack-only responses keep previous value
            if responses:
                esp_status = parse_esp_telemetry_status(responses, esp_status)

            display_wp_index = min(current_wp_index + 1, len(WAYPOINTS))
            pi_loop_dt = time.time() - loop_start

            log_nav_row(
                logger,
                waypoint_index=display_wp_index,
                waypoint_total=len(WAYPOINTS),
                target_lat=target_lat,
                target_lon=target_lon,
                lat=lat if lat is not None else 0.0,
                lon=lon if lon is not None else 0.0,
                raw_heading=raw_heading,
                heading=heading,
                gps_course=gps_course,
                gps_hdop=gps_hdop,
                gps_satellites=gps_satellites,
                gps_fix_quality=gps_fix_quality if gps_fix_quality else 0,
                offset_deg=offset_deg,
                cal_updated=cal_updated,
                bearing=bearing,
                heading_error=heading_error,
                distance=distance,
                speed=speed,
                throttle=throttle,
                steering=steering,
                ax=ax,
                ay=ay,
                az=az,
                gx=gx,
                gy=gy,
                gz=gz,
                battery_v=battery_v,
                battery_a=battery_a,
                battery_voltage_soc=battery_voltage_soc,
                battery_coulomb_soc=battery_coulomb_soc,
                battery_used_mah=battery_used_mah,
                battery_remaining_mah=battery_remaining_mah,
                battery_idle_corr=battery_idle_corr,
                cpu_temp_c=cpu_temp_c,
                mission_complete=mission_complete,
                pi_loop_dt=pi_loop_dt,
                esp_status=esp_status,
            )

            if now - last_print_time >= PRINT_INTERVAL_SEC:
                mission_state = "COMPLETE" if mission_complete else "RUN"
                print_runtime_status(
                    mission_state=mission_state,
                    waypoint_index=display_wp_index,
                    waypoint_total=len(WAYPOINTS),
                    distance=distance,
                    heading_error=heading_error,
                    speed=speed,
                    gps_hdop=gps_hdop,
                    gps_satellites=gps_satellites,
                    gps_fix_quality=gps_fix_quality if gps_fix_quality else 0,
                    throttle=throttle,
                    steering=steering,
                    battery_v=battery_v,
                    battery_a=battery_a,
                    battery_coulomb_soc=battery_coulomb_soc,
                    esp_status=esp_status,
                    pi_loop_dt=pi_loop_dt,
                    esp_response_count=len(responses),
                    cpu_temp_c=cpu_temp_c,
                )
                last_print_time = now

            # Sleep to maintain 10Hz loop rate
            elapsed = time.time() - loop_start
            if elapsed < loop_target_dt:
                time.sleep(loop_target_dt - elapsed)

    except KeyboardInterrupt:
        print("\nStopping USV navigation...")
        with suppress(Exception):
            esp.send_manual_command(0.0, 0.0)
            print("Neutral command sent to ESP.")
    finally:
        with suppress(Exception):
            logger.close()
        print("Logger closed.")


if __name__ == "__main__":
    main()
