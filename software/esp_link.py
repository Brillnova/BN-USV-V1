# esp_link.py
import json
import time
import serial


class EspLink:
    def __init__(self, port, baudrate):
        self.serial = serial.Serial(port, baudrate, timeout=1)
        self.seq = 0

    def send_manual_command(self, throttle, steering):
        """Send manual control command to ESP32."""
        message = {
            "type": "cmd_manual",
            "timestamp": time.time(),
            "seq": self.seq,
            "data": {
                "throttle": throttle,
                "steering": steering,
            },
        }

        self.seq += 1

        line = json.dumps(message) + "\n"
        self.serial.write(line.encode("utf-8"))

    def read_line(self):
        """Read one JSON line from ESP32 if available."""
        if self.serial.in_waiting > 0:
            return self.serial.readline().decode("utf-8", errors="replace").strip()
        return None

    def read_available_lines(self):
        """Read all available lines from ESP32."""
        lines = []

        while self.serial.in_waiting > 0:
            try:
                line = self.serial.readline().decode("utf-8", errors="replace").strip()
                if line:
                    lines.append(line)
            except:
                break

        return lines