# logger.py
import csv
import json
import os
from datetime import datetime


class USVLogger:
    def __init__(self, log_dir="logs"):
        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.nav_path = os.path.join(log_dir, f"nav_log_{timestamp}.csv")
        self.esp_path = os.path.join(log_dir, f"esp_protocol_{timestamp}.jsonl")

        self.nav_file = open(self.nav_path, "w", newline="", encoding="utf-8")
        self.esp_file = open(self.esp_path, "w", encoding="utf-8")

        self.nav_writer = csv.DictWriter(
            self.nav_file,
            fieldnames=[
                "time",
                "waypoint_index",
                "waypoint_total",
                "target_lat",
                "target_lon",
                "current_lat",
                "current_lon",
                "raw_heading",
                "corrected_heading",
                "gps_course",
                "gps_hdop",
                "gps_satellites",
                "gps_fix_quality",
                "mag_offset",
                "cal_updated",
                "bearing",
                "heading_error",
                "distance",
                "speed",
                "throttle",
                "steering",
                "ax",
                "ay",
                "az",
                "gx",
                "gy",
                "gz",
                "battery_v",
                "battery_a",
                "battery_voltage_soc",
                "battery_coulomb_soc",
                "battery_used_mah",
                "battery_remaining_mah",
                "battery_idle_corr",
                "cpu_temp_c",
                "mission_complete",
                "pi_loop_dt",
                "esp_mode",
                "esp_rc_mode",
                "esp_rc_ok",
                "esp_pi_fresh",
                "esp_manual_latch",
                "esp_ch1_us",
                "esp_ch2_us",
                "esp_ch4_us",
                "esp_cmd_thr_us",
                "esp_cmd_str_us",
                "esp_left_pwm",
                "esp_right_pwm",
            ],
        )

        self.nav_writer.writeheader()

        print(f"Navigation CSV log: {self.nav_path}")
        print(f"ESP JSONL log: {self.esp_path}")

    def log_nav(self, data):
        self.nav_writer.writerow(data)
        self.nav_file.flush()

    def log_esp_line(self, line):
        record = {
            "time": datetime.now().isoformat(),
            "raw": line,
        }
        self.esp_file.write(json.dumps(record, ensure_ascii=False) + "\n")
        self.esp_file.flush()

    def flush(self):
        if not self.nav_file.closed:
            self.nav_file.flush()
        if not self.esp_file.closed:
            self.esp_file.flush()

    def close(self):
        if not self.nav_file.closed:
            self.nav_file.close()
        if not self.esp_file.closed:
            self.esp_file.close()
