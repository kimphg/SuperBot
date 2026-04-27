// CubeMars Motor Controller (Servo Mode)
// Supports 1-2 motors dynamically detected via CAN scan
// Current setup: h_motor (AK60-6 V1.1, ID 104), v_motor (AK45-36, ID 111)
// Hardware: MCP2515 CAN module via SPI on Arduino Nano (CS pin 10)
// Library: mcp_can by coryjfowler (Library Manager: "MCP_CAN")
//
// ── Protocol ─────────────────────────────────────────────────────────────────
// Servo Mode - EXTENDED CAN frames (29-bit ID)
//
// Position Loop Command (Mode 4): 8 bytes
//   CAN ID = (0x04 << 8) | motorId
//   Bytes 0-3: int32_t position value * 10000 (0.01° resolution)
//   Range: -36000° to +36000°

#include <SPI.h>
#include <mcp_can.h>

#define MAX_MOTORS 2

uint8_t canData[8];
bool verbose = false;  // Forward declare for use in functions

// ── Types ─────────────────────────────────────────────────────────────────────
struct MotorState {
  int16_t angleDeg;   // -180 to +180, store as int to save space
  int16_t speedRPM;   // store as int
  int16_t currentA_mA; // current in mA (int16_t instead of float)
  uint8_t tempC;
  uint8_t error;
};

// ── MCP2515 ───────────────────────────────────────────────────────────────────
#define CAN_CS_PIN 10
MCP_CAN CAN(CAN_CS_PIN);

// ── Special commands (standard CAN frame, ext=0) ───────────────────────────────
void motorEnable(uint8_t id) {
  uint8_t d[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
  Serial.print(">> enable motor ID ");
  Serial.println(id);
  if (verbose) {
    Serial.print("[ENABLE RAW] bytes: FF FF FF FF FF FF FF FC");
    Serial.println();
  }
  CAN.sendMsgBuf(id, /*ext=*/0, 8, d);
}

void motorDisable(uint8_t id) {
  uint8_t d[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };
  CAN.sendMsgBuf(id, /*ext=*/0, 8, d);
}

void motorSetZero(uint8_t id) {
  uint8_t d[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };
  CAN.sendMsgBuf(id, /*ext=*/0, 8, d);
}

// Switch motor to servo mode (not used for MIT mode)
void setServoMode(uint8_t id) {
  Serial.print(">> switching motor ID "); Serial.print(id); Serial.println(" to Servo mode");
  uint8_t d[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
  uint32_t canId = (uint32_t)id | (0x05UL << 8);  // mode-switch command prefix
  CAN.sendMsgBuf(canId, /*ext=*/1, 8, d);
  delay(100);
}

// ── MIT Cheetah Mode ────────────────────────────────────────────────────────
#define MIT_KP_MIN  0.0f
#define MIT_KP_MAX  500.0f
#define MIT_KD_MIN  0.0f
#define MIT_KD_MAX  5.0f

struct MITConfig {
  float pMin, pMax;     // rad
  float vMin, vMax;     // rad/s
  float iffMin, iffMax; // A
  float kp, kd;
};

// Index 0 = h_motor (AK60-6 V1.1, ID 1)
// Index 1 = v_motor (if present)
MITConfig mitCfg[2] = {
  { -12.5f, 12.5f, -30.0f,  30.0f, -18.0f, 18.0f, 5.0f, 0.5f },
  { -12.5f, 12.5f, -30.0f,  30.0f, -30.0f, 30.0f, 5.0f, 0.5f },
};

static const char* motorName[2] = { "h_motor", "v_motor" };

// On Arduino Nano (AVR), int is 16-bit. Use uint32_t to avoid overflow.
static uint32_t floatToUintMIT(float x, float xMin, float xMax, uint8_t bits) {
  float span = xMax - xMin;
  if (x < xMin) x = xMin;
  else if (x > xMax) x = xMax;
  return (uint32_t)((x - xMin) * ((float)((1UL << bits) / span)));
}

void sendMITCommand(uint8_t id, uint8_t motorIdx, float posRad, float velRadS, float iff) {
  const MITConfig &cfg = mitCfg[motorIdx];
  uint32_t p    = floatToUintMIT(posRad,  cfg.pMin,   cfg.pMax,   16);
  uint32_t v    = floatToUintMIT(velRadS, cfg.vMin,   cfg.vMax,   12);
  uint32_t kp_  = floatToUintMIT(cfg.kp,  MIT_KP_MIN, MIT_KP_MAX, 12);
  uint32_t kd_  = floatToUintMIT(cfg.kd,  MIT_KD_MIN, MIT_KD_MAX, 12);
  uint32_t iff_ = floatToUintMIT(iff,     cfg.iffMin, cfg.iffMax, 12);

  // Clamp to max 16-bit / 12-bit values
  if (p > 0xFFFF) p = 0xFFFF;
  if (v > 0xFFF)  v = 0xFFF;
  if (kp_ > 0xFFF) kp_ = 0xFFF;
  if (kd_ > 0xFFF) kd_ = 0xFFF;
  if (iff_ > 0xFFF) iff_ = 0xFFF;
  uint8_t data[8];
  data[0] = p >> 8;
  data[1] = p & 0xFF;
  data[2] = v >> 4;
  data[3] = ((v & 0xF) << 4) | (kp_ >> 8);
  data[4] = kp_ & 0xFF;
  data[5] = kd_ >> 4;
  data[6] = ((kd_ & 0xF) << 4) | (iff_ >> 8);
  data[7] = iff_ & 0xFF;

  if (verbose) {
    Serial.print("[MIT CMD] p="); Serial.print(p); Serial.print(" v="); Serial.print(v);
    Serial.print(" kp="); Serial.print(kp_); Serial.print(" kd="); Serial.print(kd_);
    Serial.print(" bytes: ");
    for (int i = 0; i < 8; i++) {
      if (data[i] < 0x10) Serial.print("0");
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  CAN.sendMsgBuf(id, /*ext=*/0, 8, data);
}

// Matches official CubeMars float_to_uint implementation exactly

bool parseReply(const uint8_t *d, uint8_t len, MotorState &s) {
  if (len < 8) return false;
  int16_t rawSpeed = (int16_t)((uint16_t)d[1] | ((uint16_t)d[2] << 8));
  int16_t rawPos = (int16_t)((uint16_t)d[3] | ((uint16_t)d[4] << 8));
  s.speedRPM = (int16_t)(rawSpeed * 0.01f);
  s.angleDeg = (int16_t)(rawPos * 0.01f);
  s.currentA_mA = (int16_t)((int8_t)d[5]);  // ~1 mA per count
  s.tempC = d[6];
  s.error = d[7];
  return true;
}

// ── Motor scan ────────────────────────────────────────────────────────────────
#define SCAN_ID_MIN 1
#define SCAN_ID_MAX 127
#define SCAN_TIMEOUT_MS 100  // increased for MIT mode motors

uint8_t foundIds[MAX_MOTORS];  // Only store found motors, not 127 possible IDs
uint8_t foundCount = 0;

// Check if motor ID already found (avoids duplicates)
static bool isMotorFound(uint8_t id) {
  for (uint8_t i = 0; i < foundCount; i++) {
    if (foundIds[i] == id) return true;
  }
  return false;
}

static bool probeMotor(uint8_t id) {
  while (CAN.checkReceive() == CAN_MSGAVAIL) {  // flush
    uint8_t len;
    uint8_t buf[8];
    uint32_t rxId;
    CAN.readMsgBuf(&rxId, &len, buf);
  }

  motorEnable(id);  // enable command → motor replies with status frame

  unsigned long t0 = millis();
  while (millis() - t0 < SCAN_TIMEOUT_MS) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      uint8_t len;
      uint8_t buf[8];
      uint32_t rxId;
      CAN.readMsgBuf(&rxId, &len, buf);

      // Accept both extended (servo mode) and standard (MIT mode) frames
      bool isExt = (rxId & 0x80000000);
      uint8_t rxId_motor = isExt ? (rxId & 0xFF) : (rxId & 0x7FF);

      if (rxId_motor == id && len >= 6) {
        motorDisable(id);
        return true;
      }
    }
  }
  return false;
}

void scanMotors() {
  foundCount = 0;
  Serial.println(F("Scanning..."));

  // Phase 1: passive — catch motors already broadcasting (both servo & MIT modes)
  Serial.println(F("  Phase 1: passive..."));
  unsigned long t0 = millis();
  while (millis() - t0 < 500) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      uint8_t len;
      uint8_t buf[8];
      uint32_t rxId;
      CAN.readMsgBuf(&rxId, &len, buf);
      uint8_t id;
      bool isExt = (rxId & 0x80000000);
      if (isExt) {
        id = rxId & 0xFF;  // extended frame: motor ID in low byte
      } else {
        id = rxId & 0x7FF;  // standard frame: motor ID is the whole ID
      }
      if (id >= SCAN_ID_MIN && id <= SCAN_ID_MAX && len >= 6 && !isMotorFound(id) && foundCount < MAX_MOTORS) {
        foundIds[foundCount++] = id;
        Serial.print("  Found motor ID (passive, "); Serial.print(isExt ? "ext" : "std");
        Serial.print(" frame): ");
        Serial.println(id);
      }
    }
  }

  // Phase 2: active probe remaining IDs (only if passive didn't find anything)
  if (foundCount == 0) {
    Serial.println(F("  Phase 2: probe..."));
    for (uint8_t id = SCAN_ID_MIN; id <= SCAN_ID_MAX && foundCount < MAX_MOTORS; id++) {
      if (isMotorFound(id)) continue;
      if (probeMotor(id)) {
        foundIds[foundCount++] = id;
        Serial.print("  Found motor ID (active): ");
        Serial.println(id);
      }
    }
  }

  if (foundCount == 0) {
    Serial.println(F("No motors. Retry in 2s..."));
    delay(2000);
    scanMotors();
  } else {
    Serial.print(foundCount);
    Serial.println(" motor(s) found.");
  }
}

// ── Globals ───────────────────────────────────────────────────────────────────
uint8_t motorIds[MAX_MOTORS] = {0, 0};
uint8_t motorCount = 0;
float initialAngles[MAX_MOTORS] = {0.0f, 0.0f};
float targetAngles[MAX_MOTORS] = {-1.0e10f, -1.0e10f};
float slewAngles[MAX_MOTORS]  = {0.0f, 0.0f};

// Max degrees per 20 ms step (50 Hz).  Increased for faster response.
#define MOTOR0_STEP_DEG  2.0f   // ~100 deg/s (was 0.6 = 30 deg/s)
#define MOTOR1_STEP_DEG  6.0f   // ~300 deg/s (was 1.8 = 90 deg/s)
static const float maxStep[MAX_MOTORS] = { MOTOR0_STEP_DEG, MOTOR1_STEP_DEG };
MotorState prevStates[MAX_MOTORS] = {
  {-999, -999, -999, 255, 255},
  {-999, -999, -999, 255, 255}
};
bool motorsEnabled = false;

// ── Status reading ────────────────────────────────────────────────────────
void readMotorStatus(uint8_t motorIdx) {
  Serial.print(F("Status ")); Serial.println(motorName[motorIdx]);

  // Prompt motor with neutral MIT command
  sendMITCommand(motorIds[motorIdx], motorIdx, slewAngles[motorIdx] * DEG_TO_RAD, 0.0f, 0.0f);

  unsigned long timeout = millis() + 500;
  while (millis() < timeout) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      uint32_t rxId;
      uint8_t len;
      uint8_t buf[8];
      CAN.readMsgBuf(&rxId, &len, buf);

      bool isOurMotor = (rxId & 0x80000000) ?
        ((rxId & 0xFF) == motorIds[motorIdx]) :
        (rxId == motorIds[motorIdx]);

      if (isOurMotor) {
        Serial.print(F("raw: "));
        for (int i = 0; i < len; i++) {
          if (buf[i] < 0x10) Serial.print(F("0"));
          Serial.print(buf[i], HEX);
          Serial.print(F(" "));
        }
        Serial.println();

        if (len >= 6) {
          const MITConfig &cfg = mitCfg[motorIdx];
          uint16_t p_int = ((uint16_t)buf[1] << 8) | buf[2];
          float posDeg = (p_int / 65535.0f * (cfg.pMax - cfg.pMin) + cfg.pMin) * 57.2958f;
          uint16_t v_int = ((uint16_t)buf[3] << 4) | (buf[4] >> 4);
          float velRadS = v_int / 4095.0f * (cfg.vMax - cfg.vMin) + cfg.vMin;
          uint16_t t_int = ((uint16_t)(buf[4] & 0xF) << 8) | buf[5];
          float currA = t_int / 4095.0f * (cfg.iffMax - cfg.iffMin) + cfg.iffMin;

          Serial.print(F("pos=")); Serial.print(posDeg, 2);
          Serial.print(F(" vel=")); Serial.print(velRadS, 2);
          Serial.print(F(" I=")); Serial.println(currA, 2);

          if (len >= 8) {
            // Byte 6: temperature (typical CubeMars convention)
            // Byte 7: error code
            Serial.print(F("temp=")); Serial.print(buf[6]);
            Serial.print(F("C err=0x")); Serial.println(buf[7], HEX);
          }
        }
        return;
      }
    }
  }
  Serial.println(F("NO RESPONSE - CAN broken?"));
}

// ── Serial protocol ───────────────────────────────────────────────────────────
// Command format: "move,<angle1>,<angle2>\n"
//   angle1 → motor 0 (foundIds[0])
//   angle2 → motor 1 (foundIds[1])
// Response: "ok,<angle1>,<angle2>\n"  or  "err\n"

void processSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();

  // Silently ignore empty lines (often just CR/LF from terminal)
  if (line.length() == 0) {
    return;
  }

  Serial.print(F("got: '"));
  Serial.print(line);
  Serial.println(F("'"));

  // Check for simple commands (no comma)
  if (line == "test") {
    Serial.println(F("Serial OK"));
    return;
  } else if (line == "enable") {
    motorsEnabled = true;
    Serial.println(F("ENABLED"));
    for (uint8_t i = 0; i < motorCount; i++) {
      motorEnable(motorIds[i]);
    }
    return;
  } else if (line == "disable") {
    motorsEnabled = false;
    Serial.println(F("DISABLED"));
    for (uint8_t i = 0; i < motorCount; i++) {
      motorDisable(motorIds[i]);
    }
    return;
  } else if (line == "v") {
    verbose = !verbose;
    Serial.print("verbose: ");
    Serial.println(verbose ? "ON" : "OFF");
    return;
  } else if (line == "status") {
    for (uint8_t i = 0; i < motorCount; i++) {
      readMotorStatus(i);
    }
    return;
  } else if (line.startsWith("status,")) {
    String motorStr = line.substring(7);
    motorStr.toLowerCase();
    int idx = motorStr == "h" ? 0 : motorStr == "v" ? 1 : -1;
    if (idx >= 0 && idx < (int)motorCount) {
      readMotorStatus(idx);
    } else {
      Serial.println("err");
    }
    return;
  } else if (line == "listen") {
    Serial.println(F("Listen 5s..."));
    unsigned long endTime = millis() + 5000;
    uint32_t count = 0;
    while (millis() < endTime) {
      if (CAN.checkReceive() == CAN_MSGAVAIL) {
        uint32_t rxId;
        uint8_t len;
        uint8_t buf[8];
        CAN.readMsgBuf(&rxId, &len, buf);
        count++;
        bool isExt = (rxId & 0x80000000) ? true : false;
        uint32_t id = isExt ? (rxId & 0x1FFFFFFF) : (rxId & 0x7FF);
        Serial.print(count); Serial.print(F(":"));
        Serial.print(isExt ? F("E") : F("S"));
        Serial.print(F(" 0x")); Serial.print(id, HEX);
        Serial.print(F(" l=")); Serial.print(len);
        Serial.print(F(" "));
        for (int i = 0; i < len; i++) {
          if (buf[i] < 0x10) Serial.print(F("0"));
          Serial.print(buf[i], HEX);
        }
        Serial.println();
      }
    }
    Serial.print(F("Total: ")); Serial.println(count);
    return;
  }

  // Commands with comma
  int c1 = line.indexOf(',');
  if (c1 < 0) { Serial.println("err"); return; }
  String cmd = line.substring(0, c1);
  cmd.toLowerCase();

  if (cmd == "move") {
    int c2 = line.indexOf(',', c1 + 1);
    if (c2 < 0) { Serial.println(F("err")); return; }
    float a0 = line.substring(c1 + 1, c2).toFloat();
    float a1 = line.substring(c2 + 1).toFloat();

    if (motorCount > 0) targetAngles[0] = a0;
    if (motorCount > 1) targetAngles[1] = a1;

    Serial.print(F("ok,"));
    Serial.print(a0, 2);
    Serial.print(F(","));
    Serial.println(a1, 2);
  } else if (cmd == "pid") {
    // format: pid,<h|v>,<kp>,<kd>
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    if (c2 < 0 || c3 < 0) { Serial.println("err"); return; }
    String motor = line.substring(c1 + 1, c2);
    motor.toLowerCase();
    int idx = motor == "h" ? 0 : motor == "v" ? 1 : -1;
    if (idx < 0 || idx >= (int)motorCount) { Serial.println("err"); return; }
    mitCfg[idx].kp = constrain(line.substring(c2 + 1, c3).toFloat(), MIT_KP_MIN, MIT_KP_MAX);
    mitCfg[idx].kd = constrain(line.substring(c3 + 1).toFloat(),     MIT_KD_MIN, MIT_KD_MAX);
    Serial.print("pid,"); Serial.print(motorName[idx]);
    Serial.print(",kp="); Serial.print(mitCfg[idx].kp, 2);
    Serial.print(",kd="); Serial.println(mitCfg[idx].kd, 3);
  } else {
    Serial.println("err");
  }
}

static void readInitialAngle(uint8_t idx) {
  unsigned long t0 = millis();
  while (millis() - t0 < 200) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      uint32_t rxId; uint8_t len; uint8_t buf[8];
      CAN.readMsgBuf(&rxId, &len, buf);
      // Check if this is a servo mode response (extended frame) from our motor
      if ((rxId & 0x80000000) && (rxId & 0xFF) == motorIds[idx] && len >= 5) {
        int16_t rawPos = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
        initialAngles[idx] = rawPos * 0.01f;
        Serial.print(motorName[idx]);
        Serial.print(" initial angle: ");
        Serial.print(initialAngles[idx]);
        Serial.println(" deg");
        return;
      }
    }
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  while (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("MCP2515 init failed, retrying...");
    delay(200);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("MCP2515 ready");

  scanMotors();

  motorCount = min((uint8_t)MAX_MOTORS, foundCount);
  for (uint8_t i = 0; i < motorCount; i++) {
    motorIds[i] = foundIds[i];
    slewAngles[i] = 0.0f;
  }

  Serial.print(motorCount);
  Serial.println(F(" motor(s) found."));
  Serial.println(F("Disabled at startup. MIT MODE."));
  Serial.println(F("Cmds: enable|disable|v|status|listen|move,X,Y|pid,h,kp,kd"));
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  if (motorCount == 0) return;

  if (!motorsEnabled) {
    processSerial();  // still handle commands even when disabled
    return;
  }

  for (uint8_t i = 0; i < motorCount; i++) {
    if (targetAngles[i] < -1.0e9f) targetAngles[i] = slewAngles[i];  // hold current position
  }

  static unsigned long lastCmd = 0;
  static unsigned long lastDbg = 0;
  if (millis() - lastCmd >= 20) {  // 50 Hz
    lastCmd = millis();
    bool doLog = false;//(millis() - lastDbg >= 1000);  // log every 1 second
    if (doLog) lastDbg = millis();
    for (uint8_t i = 0; i < motorCount; i++) {
      float diff = targetAngles[i] - slewAngles[i];
      if      (diff >  maxStep[i]) diff =  maxStep[i];
      else if (diff < -maxStep[i]) diff = -maxStep[i];
      slewAngles[i] += diff;

      if (doLog) {
        Serial.print(">> MIT cmd "); Serial.print(motorName[i]);
        Serial.print(" target="); Serial.print(targetAngles[i], 2);
        Serial.print(" slew="); Serial.print(slewAngles[i], 2);
        Serial.print(" kp="); Serial.print(mitCfg[i].kp, 1);
        Serial.print(" kd="); Serial.println(mitCfg[i].kd, 2);
      }

      // MIT mode: send position with velocity towards target
      float errorDeg = targetAngles[i] - slewAngles[i];
      float targetVel = 0.0f;
      if (errorDeg > 0.1f) targetVel = 2.0f;      // 2 rad/s towards target
      else if (errorDeg < -0.1f) targetVel = -2.0f;

      if (verbose) {
        Serial.print("[DEBUG] target="); Serial.print(targetAngles[i], 2);
        Serial.print(" slew="); Serial.print(slewAngles[i], 2);
        Serial.print(" error="); Serial.print(errorDeg, 2);
        Serial.print(" vel="); Serial.println(targetVel, 2);
      }

      sendMITCommand(motorIds[i], i, slewAngles[i] * DEG_TO_RAD, targetVel * DEG_TO_RAD, 0.0f);
    }
  }

  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    uint32_t rxId; uint8_t len; uint8_t buf[8];
    CAN.readMsgBuf(&rxId, &len, buf);

    for (uint8_t i = 0; i < motorCount; i++) {
      if ((rxId & 0x80000000) && (rxId & 0xFF) == motorIds[i]) {
        if (verbose) {
          Serial.print("[DEBUG] Servo reply from motor "); Serial.print(motorIds[i]);
          Serial.print(" len="); Serial.print(len);
          Serial.print(" data: ");
          for (uint8_t j = 0; j < len; j++) {
            if (buf[j] < 0x10) Serial.print("0");
            Serial.print(buf[j], HEX);
            Serial.print(" ");
          }
          Serial.println();
        }
        break;
      }
    }
  }

  processSerial();
}
