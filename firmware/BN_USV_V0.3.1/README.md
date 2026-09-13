# BN_USV_V0.3.1 ESP32 firmware

`BN_USV_V0.3.1.ino` is the V1 ESP32-S3 reference firmware. It reads RC PWM input, drives the left and right ESC outputs, accepts serial commands from the Raspberry Pi, and emits JSON-line telemetry.

## What it does

- Reads RC steering, throttle and mode-switch channels.
- Drives separate left and right ESC PWM outputs.
- Accepts `cmd_manual` JSON-line commands from the Raspberry Pi over USB serial.
- Applies manual override, neutral re-arm and command-freshness checks.
- Moves to a neutral/failsafe state when RC or Pi commands time out.
- Reports periodic `telemetry_status` messages over serial.
- Supports optional Arduino OTA updates, disabled by default in this public release.

## Before uploading

Review the user-configuration section near the top of the sketch for your hardware:

| Setting | Default | Purpose |
| --- | --- | --- |
| `RC_CH1_PIN`, `RC_CH2_PIN`, `RC_CH4_PIN` | `4`, `5`, `6` | RC steering, throttle and mode-switch PWM input pins. |
| `ESC_L_PIN`, `ESC_R_PIN` | `1`, `2` | Left and right ESC PWM output pins. |
| `UART_BAUD` | `115200` | USB serial speed used for Raspberry Pi communication. |
| `PWM_MIN_US`, `PWM_MAX_US`, `PWM_NEU_US` | `1000`, `2000`, `1500` | ESC output range and neutral value. |
| `RC_TIMEOUT_MS`, `CH4_TIMEOUT_MS`, `PI_TIMEOUT_MS` | `300`, `500`, `500` | Input freshness limits that trigger safety behavior. |
| `OTA_SSID`, `OTA_PASSWORD` | empty | OTA is disabled while the SSID is empty. Keep any local credentials out of Git. |

The default pins and thresholds came from the V1 prototype. They are not universal values and must be checked against the actual board, receiver, ESCs and propulsion system.

## Dependencies and upload

Install the Arduino core for ESP32 and the `ESP32Servo` library in the Arduino IDE or Arduino CLI environment, then select the appropriate ESP32-S3 board and serial port before uploading. `WiFi.h` and `ArduinoOTA.h` come from the ESP32 Arduino core.

Keep OTA disabled unless you have configured local credentials in an uncommitted working copy.

## Safety check

Before connecting propulsion, confirm the configured pin mapping, neutral PWM value, RC mode behavior and serial-command timeout. Test with propellers removed or the vessel safely restrained. This is prototype firmware and does not provide a guarantee of safe operation in another hardware configuration.
