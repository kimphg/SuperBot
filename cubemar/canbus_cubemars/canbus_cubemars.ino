// CubeMars AK48-36 servo control via MCP2515 (SPI CAN) on Teensy 4.0
// MCP2515 CS pin: D10
// Library: mcp_can by coryjfowler (Library Manager: "MCP_CAN")
//
// ── Protocol ─────────────────────────────────────────────────────────────────
// Commands (host → motor): Servo mode, EXTENDED CAN frame
//   CAN ID = (0x04 << 8) | motorId  e.g. 0x049F for ID=159
//   4 bytes, int32 LE:
//     position (degrees × 10000)
//
// Status reply (motor → host): EXTENDED CAN frame
//   CAN ID = motorId | (status_code << 8)
//   8 bytes:
//     [0]   status byte
//     [1-2] int16 LE  speed    (ERPM)
//     [3-4] int16 LE  position (0.01 deg / count)
//     [5]   int8       current  (~1 mA / count, signed)
//     [6]   uint8      temperature (°C)
//     [7]   uint8      error (0 = OK)

#include <SPI.h>
#include <mcp_can.h>

uint8_t canData[8];
// ── Types ─────────────────────────────────────────────────────────────────────
struct MotorState {
  float angleDeg;
  float speedRPM;
  float currentA;
  uint8_t tempC;
  uint8_t error;
};

// ── MCP2515 ───────────────────────────────────────────────────────────────────
#define CAN_CS_PIN 10
MCP_CAN CAN(CAN_CS_PIN);

// ── Special commands (standard CAN frame, ext=0) ───────────────────────────────
void motorEnable(uint8_t id) {
  uint8_t d[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
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

// ── Servo position command ─────────────────────────────────────────────────
void sendServoPosition(uint8_t id, float angleDeg) {
  int32_t pos = (int32_t)(angleDeg * 1.0f);
  Serial.println(pos);
  uint8_t data[8];
  data[0] = (pos >> 8) & 0xFF;
  data[1] = pos & 0xFF;
  data[2] = 0xff;
  data[3] = 0xff;
  uint32_t canId = (uint32_t)id | (0x04UL << 8);
  CAN.sendMsgBuf(canId, /*ext=*/1, 8, data);
}

// Position servo: degrees → servo command
void sendAngleControl(uint8_t id, float angleDeg) {
  sendServoPosition(id, angleDeg);
}

// ── Status reply parser ───────────────────────────────────────────────────────
// Motor sends extended frame; mcp_can marks bit-31 of rxId for extended frames.
// Motor ID is in the low byte of the actual 29-bit CAN ID.
static bool isReplyFrom(uint32_t rxId, uint8_t motorId) {
  return (rxId & 0x80000000) && ((rxId & 0xFF) == motorId);
}

bool parseReply(const uint8_t *d, uint8_t len, MotorState &s) {
  if (len < 8) return false;
  int16_t rawSpeed = (int16_t)((uint16_t)d[1] | ((uint16_t)d[2] << 8));
  int16_t rawPos = (int16_t)((uint16_t)d[3] | ((uint16_t)d[4] << 8));
  s.speedRPM = rawSpeed * 0.01f;        // ERPM (approximate RPM)
  s.angleDeg = rawPos * 0.01f;          // 0.01 deg per count
  s.currentA = (int8_t)d[5] * 0.001f;  // ~1 mA per count (tune if needed)
  s.tempC = d[6];
  s.error = d[7];
  return true;
}

// ── Motor scan ────────────────────────────────────────────────────────────────
#define SCAN_ID_MIN 1
#define SCAN_ID_MAX 127
#define SCAN_TIMEOUT_MS 30

uint8_t foundIds[SCAN_ID_MAX];
uint8_t foundCount = 0;
static bool idFound[SCAN_ID_MAX + 1];

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
      if (isReplyFrom(rxId, id) && len >= 6) {
        motorDisable(id);
        return true;
      }
    }
  }
  return false;
}

void scanMotors() {
  foundCount = 0;
  memset(idFound, 0, sizeof(idFound));
  Serial.println("Scanning for AK48-36 motors...");

  // Phase 1: passive — catch motors already broadcasting
  Serial.println("  Phase 1: passive listen (500 ms)...");
  unsigned long t0 = millis();
  while (millis() - t0 < 500) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      uint8_t len;
      uint8_t buf[8];
      uint32_t rxId;
      CAN.readMsgBuf(&rxId, &len, buf);
      if (!(rxId & 0x80000000)) continue;  // ignore standard frames
      uint8_t id = rxId & 0xFF;
      if (id >= SCAN_ID_MIN && id <= SCAN_ID_MAX && len >= 6 && !idFound[id]) {
        idFound[id] = true;
        foundIds[foundCount++] = id;
        Serial.print("  Found motor ID (passive): ");
        Serial.println(id);
      }
    }
  }

  // Phase 2: active probe remaining IDs
  Serial.println("  Phase 2: active probe...");
  for (uint8_t id = SCAN_ID_MIN; id <= SCAN_ID_MAX; id++) {
    if (idFound[id]) continue;
    if (probeMotor(id)) {
      idFound[id] = true;
      foundIds[foundCount++] = id;
      Serial.print("  Found motor ID (active): ");
      Serial.println(id);
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
MotorState prevStates[MAX_MOTORS] = {
  {-999.0f, -999.0f, -999.0f, 255, 255},
  {-999.0f, -999.0f, -999.0f, 255, 255}
};
bool verbose = false;

// ── Serial protocol ───────────────────────────────────────────────────────────
// Command format: "move,<angle1>,<angle2>\n"
//   angle1 → motor 0 (foundIds[0])
//   angle2 → motor 1 (foundIds[1])
// Response: "ok,<angle1>,<angle2>\n"  or  "err\n"

void processSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();

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
        Serial.print("Motor ");
        Serial.print(motorIds[idx]);
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
    Serial.print("Enabling motor ID: ");
    Serial.println(motorIds[i]);
    motorEnable(motorIds[i]);
    delay(50);
    readInitialAngle(i);
  }

  Serial.print(motorCount);
  Serial.println(" motor(s) active.");
  Serial.println("Commands: '1:<deg>' or '2:<deg>' to set target, 'v' to toggle verbose");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  if (motorCount == 0) return;

  for (uint8_t i = 0; i < motorCount; i++) {
    if (targetAngles[i] < -1.0e9f) targetAngles[i] = initialAngles[i];
  }

  static unsigned long lastCmd = 0;
  if (millis() - lastCmd >= 20) {  // 50 Hz
    lastCmd = millis();
    for (uint8_t i = 0; i < motorCount; i++) {
      sendAngleControl(motorIds[i], targetAngles[i]);
    }
  }

  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    uint32_t rxId; uint8_t len; uint8_t buf[8];
    CAN.readMsgBuf(&rxId, &len, buf);

    Serial.print("CAN ID=0x");
    Serial.print(rxId, HEX);
    Serial.print(rxId & 0x80000000 ? " [ext]" : " [std]");
    Serial.print(" len="); Serial.print(len);
    Serial.print(" data:");
    for (uint8_t i = 0; i < len; i++) {
      Serial.print(" ");
      if (buf[i] < 0x10) Serial.print("0");
      Serial.print(buf[i], HEX);
    }
    Serial.println();

    for (uint8_t i = 0; i < motorCount; i++) {
      if (isReplyFrom(rxId, motorIds[i])) {
        MotorState st;
        if (parseReply(buf, len, st) && verbose) {
          MotorState &prev = prevStates[i];
          if (st.angleDeg != prev.angleDeg || st.speedRPM != prev.speedRPM ||
              st.currentA != prev.currentA || st.tempC != prev.tempC || st.error != prev.error) {
            Serial.print("  M"); Serial.print(motorIds[i]);
            Serial.print(" angle="); Serial.print(st.angleDeg, 1);
            Serial.print(" deg  speed="); Serial.print(st.speedRPM, 1);
            Serial.print(" RPM  current="); Serial.print(st.currentA * 1000.0f, 0);
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
