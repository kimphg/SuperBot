// CubeMars Motor Controller (MIT Cheetah Mode)
// Supports 1-2 motors dynamically detected via CAN scan
// Current setup: h_motor (AK60-6 V1.1, ID 104)
// Hardware: MCP2515 CAN module via SPI on Arduino Nano (CS pin 10)
// Library: mcp_can by coryjfowler (Library Manager: "MCP_CAN")
//
// ── Protocol ─────────────────────────────────────────────────────────────────
// MIT Cheetah Mode - STANDARD CAN frames (11-bit ID)
//
// Command (host → motor): 8 bytes
//   CAN ID = motorId (104 or 111)
//   Bytes 0-7: [p_int(16)] [v_int(12)] [kp_int(12)] [kd_int(12)] [iff_int(12)]
//   - position: ±12.5 rad
//   - velocity: ±65 rad/s (AK60-6) or ±18 rad/s (AK45-36)
//   - kp: 0-500
//   - kd: 0-5
//   - torque_ff: ±18 A (AK60-6) or ±30 A (AK45-36)
//
// Status reply (motor → host): 6 bytes
//   CAN ID = motorId
//   Bytes 0-5: [motorId] [p_int(16)] [v_int(12)] [iff_int(12)]

#include <SPI.h>
#include <mcp_can.h>

uint8_t canData[8];
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

// Switch motor from servo mode to MIT Cheetah mode
void setMITMode(uint8_t id) {
  Serial.print(">> switching motor ID "); Serial.print(id); Serial.println(" to MIT mode");
  // Send via extended servo mode CAN ID with mode-switch code
  uint8_t d[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB };
  uint32_t canId = (uint32_t)id | (0x05UL << 8);  // mode-switch command prefix
  CAN.sendMsgBuf(canId, /*ext=*/1, 8, d);
  delay(100);
}

// ── MIT Cheetah mode — both motors ───────────────────────────────────────────
// Standard CAN frame, CAN ID = motor_id.
// kp/kd common range (same for all AK series):
#define MIT_KP_MIN  0.0f
#define MIT_KP_MAX  500.0f
#define MIT_KD_MIN  0.0f
#define MIT_KD_MAX  5.0f

struct MITConfig {
  float pMin, pMax;     // rad
  float vMin, vMax;     // rad/s
  float iffMin, iffMax; // A (feedforward current)
  float kp, kd;
};

// Index 0 = h_motor (AK60-6 V1.1 KV80,  6:1 ratio, ID 104)
// Index 1 = v_motor (AK45-36    KV80, 36:1 ratio, ID 111)
MITConfig mitCfg[2] = {
  { -12.5f, 12.5f, -65.0f,  65.0f, -18.0f, 18.0f, 1.0f, 0.5f },
  { -12.5f, 12.5f, -18.0f,  18.0f, -30.0f, 30.0f, 1.0f, 1.0f },
};

static const char* motorName[2] = { "h_motor", "v_motor" };

static uint16_t floatToUint(float x, float xMin, float xMax, uint8_t bits) {
  float c = x < xMin ? xMin : (x > xMax ? xMax : x);
  return (uint16_t)((c - xMin) / (xMax - xMin) * ((1 << bits) - 1));
}

void sendMITCommand(uint8_t id, uint8_t motorIdx, float posRad, float velRadS, float iff) {
  const MITConfig &cfg = mitCfg[motorIdx];
  uint16_t p    = floatToUint(posRad,  cfg.pMin,   cfg.pMax,   16);
  uint16_t v    = floatToUint(velRadS, cfg.vMin,   cfg.vMax,   12);
  uint16_t kp_  = floatToUint(cfg.kp,  MIT_KP_MIN, MIT_KP_MAX, 12);
  uint16_t kd_  = floatToUint(cfg.kd,  MIT_KD_MIN, MIT_KD_MAX, 12);
  uint16_t iff_ = floatToUint(iff,     cfg.iffMin, cfg.iffMax, 12);
  uint8_t data[8];
  data[0] = p >> 8;
  data[1] = p & 0xFF;
  data[2] = v >> 4;
  data[3] = ((v & 0xF) << 4) | (kp_ >> 8);
  data[4] = kp_ & 0xFF;
  data[5] = kd_ >> 4;
  data[6] = ((kd_ & 0xF) << 4) | (iff_ >> 8);
  data[7] = iff_ & 0xFF;
  CAN.sendMsgBuf(id, /*ext=*/0, 8, data);
}

// ── Status reply parser ───────────────────────────────────────────────────────
// Motor sends extended frame; mcp_can marks bit-31 of rxId for extended frames.
// Motor ID is in the low byte of the actual 29-bit CAN ID.
static bool isReplyFrom(uint32_t rxId, uint8_t motorId) {
  return (rxId & 0x80000000) && ((rxId & 0xFF) == motorId);
}

// MIT reply: 6 bytes — [motorId][p_hi][p_lo][v_hi][v_lo|t_hi][t_lo]
bool parseMITReply(const uint8_t *d, uint8_t len, uint8_t motorIdx, MotorState &s) {
  if (len < 6) return false;
  const MITConfig &cfg = mitCfg[motorIdx];
  uint16_t p_int = ((uint16_t)d[1] << 8) | d[2];
  uint16_t v_int = ((uint16_t)d[3] << 4) | (d[4] >> 4);
  uint16_t t_int = ((uint16_t)(d[4] & 0xF) << 8) | d[5];
  float posRad  = p_int / 65535.0f * (cfg.pMax - cfg.pMin) + cfg.pMin;
  float velRadS = v_int / 4095.0f  * (cfg.vMax - cfg.vMin) + cfg.vMin;
  float currA = t_int / 4095.0f * (cfg.iffMax - cfg.iffMin) + cfg.iffMin;
  s.angleDeg = (int16_t)(posRad * (180.0f / PI));
  s.speedRPM = (int16_t)(velRadS * (60.0f / (2.0f * PI)));
  s.currentA_mA = (int16_t)(currA * 1000.0f);
  s.tempC = 0;
  s.error = 0;
  return true;
}

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
  Serial.println("Scanning for CubeMars motors...");

  // Phase 1: passive — catch motors already broadcasting (both servo & MIT modes)
  Serial.println("  Phase 1: passive listen (500 ms)...");
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
    Serial.println("  Phase 2: active probe...");
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
    Serial.println("No motors found. Retrying in 2 s...");
    delay(2000);
    scanMotors();
  } else {
    Serial.print(foundCount);
    Serial.println(" motor(s) found.");
  }
}

// ── Globals ───────────────────────────────────────────────────────────────────
#define MAX_MOTORS 2

uint8_t motorIds[MAX_MOTORS] = {0, 0};
uint8_t motorCount = 0;
float initialAngles[MAX_MOTORS] = {0.0f, 0.0f};
float targetAngles[MAX_MOTORS] = {-1.0e10f, -1.0e10f};
float slewAngles[MAX_MOTORS]  = {0.0f, 0.0f};

// Max degrees per 20 ms step (50 Hz).  Motor 104 is 1/3 the speed of motor 111.
#define MOTOR0_STEP_DEG  0.6f   // ~30 deg/s
#define MOTOR1_STEP_DEG  1.8f   // ~90 deg/s
static const float maxStep[MAX_MOTORS] = { MOTOR0_STEP_DEG, MOTOR1_STEP_DEG };
MotorState prevStates[MAX_MOTORS] = {
  {-999, -999, -999, 255, 255},
  {-999, -999, -999, 255, 255}
};
bool verbose = false;
bool motorsEnabled = false;

// ── Serial protocol ───────────────────────────────────────────────────────────
// Command format: "move,<angle1>,<angle2>\n"
//   angle1 → motor 0 (foundIds[0])
//   angle2 → motor 1 (foundIds[1])
// Response: "ok,<angle1>,<angle2>\n"  or  "err\n"

void processSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();

  // Check for simple commands (no comma)
  if (line == "enable") {
    motorsEnabled = true;
    Serial.println("Motors ENABLED - be careful!");
    for (uint8_t i = 0; i < motorCount; i++) {
      motorEnable(motorIds[i]);
    }
    return;
  } else if (line == "disable") {
    motorsEnabled = false;
    Serial.println("Motors DISABLED");
    for (uint8_t i = 0; i < motorCount; i++) {
      motorDisable(motorIds[i]);
    }
    return;
  } else if (line == "v") {
    verbose = !verbose;
    Serial.print("verbose: ");
    Serial.println(verbose ? "ON" : "OFF");
    return;
  } else if (line == "mit") {
    Serial.println("Switching all motors to MIT mode...");
    for (uint8_t i = 0; i < motorCount; i++) {
      setMITMode(motorIds[i]);
    }
    delay(200);
    Serial.println("Motors switched. Send 'enable' to activate.");
    return;
  }

  // Commands with comma
  int c1 = line.indexOf(',');
  if (c1 < 0) { Serial.println("err"); return; }
  String cmd = line.substring(0, c1);
  cmd.toLowerCase();

  if (cmd == "move") {
    int c2 = line.indexOf(',', c1 + 1);
    if (c2 < 0) { Serial.println("err"); return; }
    float a0 = line.substring(c1 + 1, c2).toFloat();
    float a1 = line.substring(c2 + 1).toFloat();
    if (motorCount > 0) targetAngles[0] = a0;
    if (motorCount > 1) targetAngles[1] = a1;
    Serial.print("ok,");
    Serial.print(a0, 2);
    Serial.print(",");
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
      if (isReplyFrom(rxId, motorIds[idx]) && len >= 5) {
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
  Serial.println(" motor(s) found.");
  Serial.println("⚠ Motors disabled at startup for safety.");
  Serial.println("Commands: 'enable' | 'disable' | 'move,<deg>,<deg>' | 'pid,<h|v>,<kp>,<kd>' | 'v' verbose");
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
    bool doLog = (millis() - lastDbg >= 1000);  // log every 1 second
    if (doLog) lastDbg = millis();
    for (uint8_t i = 0; i < motorCount; i++) {
      float diff = targetAngles[i] - slewAngles[i];
      if      (diff >  maxStep[i]) diff =  maxStep[i];
      else if (diff < -maxStep[i]) diff = -maxStep[i];
      slewAngles[i] += diff;
      if (doLog) {
        Serial.print(">> MIT cmd "); Serial.print(motorName[i]);
        Serial.print(" pos="); Serial.print(slewAngles[i] * DEG_TO_RAD, 2); Serial.print(" kp="); Serial.print(mitCfg[i].kp, 1);
        Serial.print(" kd="); Serial.println(mitCfg[i].kd, 2);
      }
      sendMITCommand(motorIds[i], i, slewAngles[i] * DEG_TO_RAD, 0.0f, 0.0f);
    }
  }

  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    uint32_t rxId; uint8_t len; uint8_t buf[8];
    CAN.readMsgBuf(&rxId, &len, buf);

    for (uint8_t i = 0; i < motorCount; i++) {
      if (!(rxId & 0x80000000) && rxId == motorIds[i]) {
        MotorState st;
        if (parseMITReply(buf, len, i, st) && verbose) {
          MotorState &prev = prevStates[i];
          if (st.angleDeg != prev.angleDeg || st.speedRPM != prev.speedRPM ||
              st.currentA_mA != prev.currentA_mA || st.tempC != prev.tempC || st.error != prev.error) {
            Serial.print("  "); Serial.print(motorName[i]);
            Serial.print(" angle="); Serial.print(st.angleDeg);
            Serial.print(" deg  speed="); Serial.print(st.speedRPM);
            Serial.print(" RPM  current="); Serial.print(st.currentA_mA);
            Serial.print(" mA  temp="); Serial.print(st.tempC);
            Serial.print(" C  err=0x"); Serial.println(st.error, HEX);
            prev = st;
          }
        }
        break;
      }
    }
  }

  processSerial();
}
