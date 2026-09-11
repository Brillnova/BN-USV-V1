#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// =====================
// Debug configuration
// =====================
#define DEBUG_MODE 0  // 1 = enable debug prints, 0 = disable for Pi communication

#if DEBUG_MODE
  #define DBG_PRINT(x)     Serial.print(x)
  #define DBG_PRINTLN(x)   Serial.println(x)
  #define DBG_PRINTF(...)  Serial.printf(__VA_ARGS__)
#else
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
  #define DBG_PRINTF(...)
#endif

// =====================
// User configuration
// =====================

// RC PWM input pins
static const int RC_CH1_PIN = 4;  // Steering PWM input
static const int RC_CH2_PIN = 5;  // Throttle PWM input
static const int RC_CH4_PIN = 6;  // Mode switch PWM input

// ESC output pins
static const int ESC_L_PIN = 1;  // Left ESC PWM output
static const int ESC_R_PIN = 2;  // Right ESC PWM output

// USB Serial baud rate for Raspberry Pi <-> ESP32 communication
static const int UART_BAUD = 115200;

// OTA configuration
// Leave OTA_SSID empty to disable OTA and keep USB-only operation.
static const char *OTA_SSID = "";  // Set your network name to enable OTA.
static const char *OTA_PASSWORD = "";  // Keep local; do not commit credentials.
static const char *OTA_HOSTNAME = "BN-USV-ESP32S3";
static bool otaEnabled = false;

// PWM limits
static const int PWM_MIN_US = 1000;
static const int PWM_MAX_US = 2000;
static const int PWM_NEU_US = 1500;

// RC processing
static const int DEADBAND_US = 20;

// Safety timeouts
static const uint32_t RC_TIMEOUT_MS = 300;
static const uint32_t CH4_TIMEOUT_MS = 500;
static const uint32_t PI_TIMEOUT_MS = 500;  // Protocol baseline: >500 ms -> failsafe
static const uint32_t STATUS_PERIOD_MS = 500;  // JSON status telemetry period

// RC override latch
static const int OVERRIDE_THRESH_US = 220;
static const int AUTO_ARM_DEADBAND_US = 60;

// Serial RX buffer
static const size_t RX_LINE_MAX = 256;

// Slew rate limiter (us per second)
static const float SLEW_ACCEL_US_PER_S = 250.0f;
static const float SLEW_DECEL_US_PER_S = 2000.0f;

// =====================
// Global types
// =====================

enum ControlMode {
  MODE_MANUAL = 0,
  MODE_AUTO,
  MODE_KILL,
  MODE_FAILSAFE
};

// =====================
// Globals
// =====================

Servo escL;
Servo escR;

static int escL_out_us = PWM_NEU_US;
static int escR_out_us = PWM_NEU_US;
static uint32_t lastSlewMs = 0;

static bool manualLatch = false;
static ControlMode lastRcMode = MODE_MANUAL;

// RC capture state
volatile uint32_t ch1Rise = 0;
volatile uint32_t ch2Rise = 0;
volatile uint32_t ch4Rise = 0;
volatile uint16_t ch1Us = PWM_NEU_US;
volatile uint16_t ch2Us = PWM_NEU_US;
volatile uint16_t ch4Us = PWM_NEU_US;
volatile uint32_t ch1LastOk = 0;
volatile uint32_t ch2LastOk = 0;
volatile uint32_t ch4LastOk = 0;

// Pi command state
static uint16_t piThrUs = PWM_NEU_US;
static uint16_t piStrUs = PWM_NEU_US;
static float piThrNorm = 0.0f;
static float piStrNorm = 0.0f;
static long piCmdSeq = -1;
static uint32_t piLastRxMs = 0;

// =====================
// Utility functions
// =====================

static inline int clampInt(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

static inline float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

static inline int applyDeadband(int value, int deadband) {
  if (abs(value) <= deadband) return 0;
  return value;
}

static uint16_t normalizedToPwm(float value) {
  value = clampFloat(value, -1.0f, 1.0f);
  int pwm = PWM_NEU_US + (int)(value * 500.0f);
  return (uint16_t)clampInt(pwm, PWM_MIN_US, PWM_MAX_US);
}

static int applySlewAroundNeutral(int current, int target, float accelRate, float decelRate, float dt_s) {
  if (dt_s <= 0.0f) return current;
  if (current == target) return current;

  int currentDev = current - PWM_NEU_US;
  int targetDev = target - PWM_NEU_US;

  bool movingAwayFromNeutral = abs(targetDev) > abs(currentDev);
  float rate = movingAwayFromNeutral ? accelRate : decelRate;

  int maxStep = (int)(rate * dt_s + 0.5f);
  if (maxStep < 1) maxStep = 1;

  int diff = target - current;
  if (diff > 0) {
    return current + min(diff, maxStep);
  }

  return current - min(-diff, maxStep);
}

static void writeNeutralImmediate() {
  escL_out_us = PWM_NEU_US;
  escR_out_us = PWM_NEU_US;
  escL.writeMicroseconds(PWM_NEU_US);
  escR.writeMicroseconds(PWM_NEU_US);
}

// =====================
// RC interrupt handlers
// =====================

void IRAM_ATTR isr_ch1() {
  if (digitalRead(RC_CH1_PIN)) {
    ch1Rise = micros();
  } else {
    uint32_t width = micros() - ch1Rise;
    if (width >= 900 && width <= 2100) {
      ch1Us = (uint16_t)width;
      ch1LastOk = millis();
    }
  }
}

void IRAM_ATTR isr_ch2() {
  if (digitalRead(RC_CH2_PIN)) {
    ch2Rise = micros();
  } else {
    uint32_t width = micros() - ch2Rise;
    if (width >= 900 && width <= 2100) {
      ch2Us = (uint16_t)width;
      ch2LastOk = millis();
    }
  }
}

void IRAM_ATTR isr_ch4() {
  if (digitalRead(RC_CH4_PIN)) {
    ch4Rise = micros();
  } else {
    uint32_t width = micros() - ch4Rise;
    if (width >= 900 && width <= 2100) {
      ch4Us = (uint16_t)width;
      ch4LastOk = millis();
    }
  }
}


static const char *modeToString(ControlMode mode);

// =====================
// JSON response helpers
// =====================

static void sendAck(long seq, float throttle, float steering, uint16_t thrUs, uint16_t strUs) {
  Serial.print("{\"type\":\"ack\",");
  Serial.print("\"timestamp\":");
  Serial.print(millis() / 1000.0f, 3);
  Serial.print(",\"seq\":");
  Serial.print(seq);
  Serial.print(",\"data\":{");
  Serial.print("\"accepted\":true,");
  Serial.print("\"throttle\":");
  Serial.print(throttle, 3);
  Serial.print(",\"steering\":");
  Serial.print(steering, 3);
  Serial.print(",\"thr_us\":");
  Serial.print(thrUs);
  Serial.print(",\"str_us\":");
  Serial.print(strUs);
  Serial.println("}}");
}

static void sendReject(long seq, const char *reason) {
  Serial.print("{\"type\":\"ack\",");
  Serial.print("\"timestamp\":");
  Serial.print(millis() / 1000.0f, 3);
  Serial.print(",\"seq\":");
  Serial.print(seq);
  Serial.print(",\"data\":{");
  Serial.print("\"accepted\":false,");
  Serial.print("\"reason\":\"");
  Serial.print(reason);
  Serial.println("\"}}");
}

static void sendStatus(ControlMode mode, ControlMode rcMode, bool rcOk, bool piFresh,
                       uint16_t ch1, uint16_t ch2, uint16_t ch4,
                       uint16_t cmdThr, uint16_t cmdStr,
                       int leftOutUs, int rightOutUs) {
  Serial.print("{\"type\":\"telemetry_status\",");
  Serial.print("\"timestamp\":");
  Serial.print(millis() / 1000.0f, 3);
  Serial.print(",\"seq\":");
  Serial.print(piCmdSeq);
  Serial.print(",\"data\":{");
  Serial.print("\"mode\":\"");
  Serial.print(modeToString(mode));
  Serial.print("\",");
  Serial.print("\"rc_mode\":\"");
  Serial.print(modeToString(rcMode));
  Serial.print("\",");
  Serial.print("\"rc_ok\":");
  Serial.print(rcOk ? "true" : "false");
  Serial.print(",\"pi_fresh\":");
  Serial.print(piFresh ? "true" : "false");
  Serial.print(",\"manual_latch\":");
  Serial.print(manualLatch ? "true" : "false");
  Serial.print(",\"ch1_us\":");
  Serial.print(ch1);
  Serial.print(",\"ch2_us\":");
  Serial.print(ch2);
  Serial.print(",\"ch4_us\":");
  Serial.print(ch4);
  Serial.print(",\"cmd_thr_us\":");
  Serial.print(cmdThr);
  Serial.print(",\"cmd_str_us\":");
  Serial.print(cmdStr);
  Serial.print(",\"left_pwm\":");
  Serial.print(leftOutUs);
  Serial.print(",\"right_pwm\":");
  Serial.print(rightOutUs);
  Serial.println("}}");
}

// =====================
// Pi command parsing
// =====================

static bool extractJsonNumber(const String &line, const char *key, float &outValue) {
  String quotedKey = String("\"") + key + "\"";
  int keyIndex = line.indexOf(quotedKey);
  if (keyIndex < 0) return false;

  int colonIndex = line.indexOf(':', keyIndex + quotedKey.length());
  if (colonIndex < 0) return false;

  int start = colonIndex + 1;
  while (start < (int)line.length() && isspace((unsigned char)line[start])) {
    start++;
  }

  int end = start;
  while (end < (int)line.length()) {
    char c = line[end];
    if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.') {
      end++;
    } else {
      break;
    }
  }

  if (end == start) return false;

  outValue = line.substring(start, end).toFloat();
  return true;
}

static bool parseJsonManualCommand(const String &line, uint16_t &thrOut, uint16_t &strOut, float &thrNormOut, float &strNormOut, long &seqOut) {
  if (line.indexOf("\"type\"") < 0 || line.indexOf("cmd_manual") < 0) {
    return false;
  }

  float throttle = 0.0f;
  float steering = 0.0f;
  float seqFloat = -1.0f;

  extractJsonNumber(line, "seq", seqFloat);
  seqOut = (long)seqFloat;

  if (!extractJsonNumber(line, "throttle", throttle)) return false;
  if (!extractJsonNumber(line, "steering", steering)) return false;

  if (throttle < -1.0f || throttle > 1.0f) return false;
  if (steering < -1.0f || steering > 1.0f) return false;

  thrNormOut = throttle;
  strNormOut = steering;
  thrOut = normalizedToPwm(throttle);
  strOut = normalizedToPwm(steering);
  return true;
}

static bool parseLegacyCsvCommand(const String &line, uint16_t &thrOut, uint16_t &strOut, float &thrNormOut, float &strNormOut, long &seqOut) {
  int commaIndex = line.indexOf(',');
  if (commaIndex < 0) return false;

  String thrText = line.substring(0, commaIndex);
  String strText = line.substring(commaIndex + 1);
  thrText.trim();
  strText.trim();

  if (thrText.length() == 0 || strText.length() == 0) return false;

  long throttle = thrText.toInt();
  long steering = strText.toInt();

  if (throttle < PWM_MIN_US || throttle > PWM_MAX_US) return false;
  if (steering < PWM_MIN_US || steering > PWM_MAX_US) return false;

  thrOut = (uint16_t)throttle;
  strOut = (uint16_t)steering;
  thrNormOut = ((float)thrOut - PWM_NEU_US) / 500.0f;
  strNormOut = ((float)strOut - PWM_NEU_US) / 500.0f;
  seqOut = -1;
  return true;
}

static bool parsePiLine(const String &line, uint16_t &thrOut, uint16_t &strOut,
                        float &thrNormOut, float &strNormOut, long &seqOut) {
  if (parseJsonManualCommand(line, thrOut, strOut, thrNormOut, strNormOut, seqOut)) return true;

  // Keep legacy CSV support for bench testing only.
  if (parseLegacyCsvCommand(line, thrOut, strOut, thrNormOut, strNormOut, seqOut)) return true;

  return false;
}

// =====================
// Mode handling
// =====================

static ControlMode getModeFromCH4(uint16_t ch4) {
  if (ch4 < 1300) return MODE_MANUAL;
  if (ch4 < 1700) return MODE_AUTO;
  return MODE_KILL;
}

static const char *modeToString(ControlMode mode) {
  switch (mode) {
    case MODE_MANUAL: return "MANUAL";
    case MODE_AUTO: return "AUTO";
    case MODE_KILL: return "KILL";
    case MODE_FAILSAFE: return "FAILSAFE";
    default: return "UNKNOWN";
  }
}

// =====================
// Arduino lifecycle
// =====================


// =====================
// OTA update
// =====================

void setupOTA() {
  if (strlen(OTA_SSID) == 0) {
    otaEnabled = false;
    DBG_PRINTLN("OTA disabled: OTA_SSID is empty.");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(OTA_HOSTNAME);
  WiFi.begin(OTA_SSID, OTA_PASSWORD);

  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < 10000) {
    delay(250);
  }

  if (WiFi.status() != WL_CONNECTED) {
    otaEnabled = false;
    DBG_PRINTLN("OTA disabled: WiFi connection failed.");
    return;
  }

  ArduinoOTA.setHostname(OTA_HOSTNAME);

  ArduinoOTA.onStart([]() {
    // Keep propulsion safe during firmware update.
    writeNeutralImmediate();
    DBG_PRINTLN("OTA update started.");
  });

  ArduinoOTA.onEnd([]() {
    writeNeutralImmediate();
    DBG_PRINTLN("OTA update finished.");
  });

  ArduinoOTA.onError([](ota_error_t error) {
    writeNeutralImmediate();
    DBG_PRINTF("OTA error: %u\n", error);
  });

  ArduinoOTA.begin();
  otaEnabled = true;

  DBG_PRINT("OTA ready. IP: ");
  DBG_PRINTLN(WiFi.localIP());
}

void setup() {
  Serial.begin(UART_BAUD);
  delay(200);

  setupOTA();

  pinMode(RC_CH1_PIN, INPUT);
  pinMode(RC_CH2_PIN, INPUT);
  pinMode(RC_CH4_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(RC_CH1_PIN), isr_ch1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH2_PIN), isr_ch2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH4_PIN), isr_ch4, CHANGE);

  escL.setPeriodHertz(50);
  escR.setPeriodHertz(50);
  escL.attach(ESC_L_PIN, PWM_MIN_US, PWM_MAX_US);
  escR.attach(ESC_R_PIN, PWM_MIN_US, PWM_MAX_US);

  writeNeutralImmediate();
  delay(2000);  // Give ESCs time to arm at neutral.

  piLastRxMs = 0;

  DBG_PRINTLN("BN-USV ESP32 controller started.");
}

void loop() {
  if (otaEnabled) {
    ArduinoOTA.handle();
  }

  // =====================
  // 1) Read Pi USB Serial lines
  // =====================
  static String rxLine;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n') {
      uint16_t thrTmp = PWM_NEU_US;
      uint16_t strTmp = PWM_NEU_US;
      float thrNormTmp = 0.0f;
      float strNormTmp = 0.0f;
      long seqTmp = -1;

      if (parsePiLine(rxLine, thrTmp, strTmp, thrNormTmp, strNormTmp, seqTmp)) {
        piThrUs = thrTmp;
        piStrUs = strTmp;
        piThrNorm = thrNormTmp;
        piStrNorm = strNormTmp;
        piCmdSeq = seqTmp;
        piLastRxMs = millis();
        sendAck(piCmdSeq, piThrNorm, piStrNorm, piThrUs, piStrUs);
      } else if (rxLine.length() > 0) {
        sendReject(-1, "PARSE_ERROR");
      }

      rxLine = "";
    } else if (c != '\r') {
      if (rxLine.length() < RX_LINE_MAX) {
        rxLine += c;
      } else {
        rxLine = "";
      }
    }
  }

  // =====================
  // 2) Snapshot RC inputs
  // =====================
  uint32_t nowMs = millis();

  uint16_t ch1 = ch1Us;
  uint16_t ch2 = ch2Us;
  uint16_t ch4 = ch4Us;

  bool rcOk = true;
  if (nowMs - ch1LastOk > RC_TIMEOUT_MS) rcOk = false;
  if (nowMs - ch2LastOk > RC_TIMEOUT_MS) rcOk = false;
  if (nowMs - ch4LastOk > CH4_TIMEOUT_MS) rcOk = false;

  if (!rcOk) {
    writeNeutralImmediate();
    DBG_PRINTLN("MODE=FAILSAFE | REASON=RC_TIMEOUT");
    delay(10);
    return;
  }

  // =====================
  // 3) Determine RC mode and safety priority
  // =====================
  ControlMode rcMode = getModeFromCH4(ch4);

  if (rcMode == MODE_KILL) {
    manualLatch = false;
    writeNeutralImmediate();
    DBG_PRINTLN("MODE=KILL | OUTPUT=NEUTRAL");
    delay(10);
    return;
  }

  int steerDev = (int)ch1 - PWM_NEU_US;
  int thrDev = (int)ch2 - PWM_NEU_US;

  bool rcOverride = (abs(steerDev) > OVERRIDE_THRESH_US) || (abs(thrDev) > OVERRIDE_THRESH_US);

  if (rcMode == MODE_AUTO && rcOverride) {
    manualLatch = true;
  }

  if (rcMode == MODE_MANUAL) {
    manualLatch = false;
  }

  bool autoArmRequest = false;
  if (lastRcMode == MODE_MANUAL && rcMode == MODE_AUTO) {
    bool sticksNearNeutral =
        (abs(steerDev) < AUTO_ARM_DEADBAND_US) &&
        (abs(thrDev) < AUTO_ARM_DEADBAND_US);

    if (sticksNearNeutral) {
      autoArmRequest = true;
    } else {
      manualLatch = true;
    }
  }

  ControlMode mode = rcMode;
  if (manualLatch && rcMode == MODE_AUTO) {
    mode = MODE_MANUAL;
  }
  if (autoArmRequest) {
    manualLatch = false;
    mode = MODE_AUTO;
  }

  lastRcMode = rcMode;

  // =====================
  // 4) Choose command source
  // =====================
  uint16_t cmdThr = PWM_NEU_US;
  uint16_t cmdStr = PWM_NEU_US;

  bool piFresh = (piLastRxMs != 0) && ((nowMs - piLastRxMs) <= PI_TIMEOUT_MS);

  if (mode == MODE_MANUAL) {
    cmdThr = ch2;
    cmdStr = ch1;
  } else if (mode == MODE_AUTO) {
    if (piFresh) {
      cmdThr = piThrUs;
      cmdStr = piStrUs;
    } else {
      mode = MODE_FAILSAFE;
      cmdThr = PWM_NEU_US;
      cmdStr = PWM_NEU_US;
    }
  }

  // =====================
  // 5) Mix throttle and steering
  // =====================
  int thr = (int)cmdThr - PWM_NEU_US;
  int str = (int)cmdStr - PWM_NEU_US;

  thr = applyDeadband(thr, DEADBAND_US);
  str = applyDeadband(str, DEADBAND_US);

  int leftUs = PWM_NEU_US + thr + str;
  int rightUs = PWM_NEU_US + thr - str;

  leftUs = clampInt(leftUs, PWM_MIN_US, PWM_MAX_US);
  rightUs = clampInt(rightUs, PWM_MIN_US, PWM_MAX_US);

  // =====================
  // 6) Slew limit and output
  // =====================
  if (lastSlewMs == 0) lastSlewMs = nowMs;
  float dt_s = (nowMs - lastSlewMs) / 1000.0f;
  lastSlewMs = nowMs;

  bool forceNeutral = (leftUs == PWM_NEU_US && rightUs == PWM_NEU_US);

  if (forceNeutral || mode == MODE_FAILSAFE) {
    escL_out_us = PWM_NEU_US;
    escR_out_us = PWM_NEU_US;
  } else {
    escL_out_us = applySlewAroundNeutral(
        escL_out_us,
        leftUs,
        SLEW_ACCEL_US_PER_S,
        SLEW_DECEL_US_PER_S,
        dt_s);

    escR_out_us = applySlewAroundNeutral(
        escR_out_us,
        rightUs,
        SLEW_ACCEL_US_PER_S,
        SLEW_DECEL_US_PER_S,
        dt_s);
  }

  escL.writeMicroseconds(escL_out_us);
  escR.writeMicroseconds(escR_out_us);

  // =====================
  // 7) JSON status telemetry
  // =====================
  static uint32_t lastStatusMs = 0;
  if (nowMs - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = nowMs;
    sendStatus(
        mode,
        rcMode,
        rcOk,
        piFresh,
        ch1,
        ch2,
        ch4,
        cmdThr,
        cmdStr,
        escL_out_us,
        escR_out_us);
  }

  delay(10);
}
