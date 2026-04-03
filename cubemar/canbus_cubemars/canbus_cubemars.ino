// CubeMars AK48-36 servo mode control via MCP2515 (SPI CAN) on Teensy 4.0
// MCP2515 CS pin: D10
// Library required: mcp_can by coryjfowler (Library Manager: "MCP_CAN")
//
// ── Extended CAN frame protocol ───────────────────────────────────────────────
// Commands  (host → motor):
//   Extended CAN ID = motorId  (29-bit, same numeric value as motor ID)
//   8 bytes:
//     [0]   command type:  0x0A = position, 0x0B = speed, 0x0C = torque
//     [1-4] float32 target (little-endian): degrees / RPM / Amps
//     [5-6] int16 speed limit in RPM (little-endian, position mode only)
//     [7]   0x00
//
// Status reply (motor → host):
//   Extended CAN ID = (0x29 << 8) | motorId   e.g. 0x296F for ID=111
//   8 bytes:
//     [0]   status  0xFF = enabled/running
//     [1-2] int16 LE position  (unit: 0.1 deg)
//     [3-4] int16 LE velocity  (unit: RPM)
//     [5]   uint8  current     (unit: 0.1 A)
//     [6]   uint8  temperature (°C)
//     [7]   uint8  error code  (0 = OK)

#include <SPI.h>
#include <mcp_can.h>

// ── Types ─────────────────────────────────────────────────────────────────────
struct MotorState {
  float   angleDeg;
  float   speedRPM;
  float   currentA;
  uint8_t tempC;
  uint8_t error;
};

// ── MCP2515 ───────────────────────────────────────────────────────────────────
#define CAN_CS_PIN  10
MCP_CAN CAN(CAN_CS_PIN);

// ── Helpers ───────────────────────────────────────────────────────────────────
static void packFloat(uint8_t *dst, float v) { memcpy(dst, &v, 4); }
static void packI16  (uint8_t *dst, int16_t v) {
  dst[0] = (uint8_t)(v & 0xFF);
  dst[1] = (uint8_t)((v >> 8) & 0xFF);
}

// ── Command helpers ───────────────────────────────────────────────────────────
// All commands use extended CAN frames (ext = 1).
// The reply CAN ID is (0x29 << 8) | motorId; we identify the motor by
// checking (rxId & 0xFF) == motorId when bit 31 (extended flag) is set.

void motorEnable(uint8_t id) {
  uint8_t d[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
  CAN.sendMsgBuf(id, /*ext=*/1, 8, d);
}

void motorDisable(uint8_t id) {
  uint8_t d[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
  CAN.sendMsgBuf(id, /*ext=*/1, 8, d);
}

void motorSetZero(uint8_t id) {
  uint8_t d[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
  CAN.sendMsgBuf(id, /*ext=*/1, 8, d);
}

void sendPositionCmd(uint8_t id, float angleDeg, int16_t maxRPM = 200) {
  uint8_t d[8] = {};
  d[0] = 0x0A;
  packFloat(&d[1], angleDeg);
  packI16  (&d[5], maxRPM);
  d[7] = 0x00;
  CAN.sendMsgBuf(id, /*ext=*/1, 8, d);
}

// ── Reply parser ──────────────────────────────────────────────────────────────
bool parseReply(const uint8_t *d, uint8_t len, MotorState &s) {
  if (len < 7) return false;
  int16_t rawPos = (int16_t)((uint16_t)d[1] | ((uint16_t)d[2] << 8));
  int16_t rawVel = (int16_t)((uint16_t)d[3] | ((uint16_t)d[4] << 8));
  s.angleDeg = rawPos * 0.1f;          // 0.1 deg per count
  s.speedRPM = (float)rawVel;
  s.currentA = d[5] * 0.1f;           // 0.1 A per count
  s.tempC    = d[6];
  s.error    = (len >= 8) ? d[7] : 0;
  return true;
}

// Check if rxId (from mcp_can) is an extended-frame reply from motorId.
// mcp_can sets bit 31 for extended frames; the motor puts its ID in bits [7:0].
static bool isReplyFrom(uint32_t rxId, uint8_t motorId) {
  return (rxId & 0x80000000) && ((rxId & 0xFF) == motorId);
}

// ── Motor scan ────────────────────────────────────────────────────────────────
#define SCAN_ID_MIN      1
#define SCAN_ID_MAX      127
#define SCAN_TIMEOUT_MS  100

uint8_t foundIds[SCAN_ID_MAX];
uint8_t foundCount = 0;
static bool idFound[SCAN_ID_MAX + 1];

static bool probeMotor(uint8_t id) {
  // Flush pending frames
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    uint8_t len; uint8_t buf[8]; uint32_t rxId;
    CAN.readMsgBuf(&rxId, &len, buf);
  }

  sendPositionCmd(id, 0.0f, 0);   // elicit a status reply

  unsigned long t0 = millis();
  while (millis() - t0 < SCAN_TIMEOUT_MS) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      uint8_t len; uint8_t buf[8]; uint32_t rxId;
      CAN.readMsgBuf(&rxId, &len, buf);
      if (isReplyFrom(rxId, id) && len >= 6) return true;
    }
  }
  return false;
}

void scanMotors() {
  foundCount = 0;
  memset(idFound, 0, sizeof(idFound));
  Serial.println("Scanning for AK48-36 motors (extended CAN)...");

  // Phase 1: passive — catch motors already broadcasting
  Serial.println("  Phase 1: passive listen (500 ms)...");
  unsigned long t0 = millis();
  while (millis() - t0 < 500) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      uint8_t len; uint8_t buf[8]; uint32_t rxId;
      CAN.readMsgBuf(&rxId, &len, buf);
      if (!(rxId & 0x80000000)) continue;          // ignore standard frames
      uint8_t id = rxId & 0xFF;
      if (id >= SCAN_ID_MIN && id <= SCAN_ID_MAX && len >= 6 && !idFound[id]) {
        idFound[id] = true;
        foundIds[foundCount++] = id;
        Serial.print("  Found motor ID (passive): "); Serial.println(id);
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
      Serial.print("  Found motor ID (active): "); Serial.println(id);
    }
  }

  if (foundCount == 0) {
    Serial.println("No motors found. Retrying in 2 s...");
    delay(2000);
    scanMotors();
  } else {
    Serial.print(foundCount); Serial.println(" motor(s) found.");
  }
}

// ── Globals ───────────────────────────────────────────────────────────────────
uint8_t activeMotorId = 0;

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

  if (foundCount > 0) {
    activeMotorId = foundIds[0];
    Serial.print("Using motor ID: "); Serial.println(activeMotorId);
    delay(100);
    motorEnable(activeMotorId);
    Serial.println("Motor enabled");
  }
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  if (activeMotorId == 0) return;

  static float targetAngle = 0.0f;
  static unsigned long lastCmd = 0;

  if (millis() - lastCmd >= 20) {    // 50 Hz
    lastCmd = millis();
    sendPositionCmd(activeMotorId, targetAngle, /*maxRPM=*/200);
  }

  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    uint32_t rxId; uint8_t len; uint8_t buf[8];
    CAN.readMsgBuf(&rxId, &len, buf);

    // Raw frame
    Serial.print("CAN ID=0x"); Serial.print(rxId, HEX);
    Serial.print(rxId & 0x80000000 ? " [ext]" : " [std]");
    Serial.print(" len="); Serial.print(len);
    Serial.print(" data:");
    for (uint8_t i = 0; i < len; i++) {
      Serial.print(" ");
      if (buf[i] < 0x10) Serial.print("0");
      Serial.print(buf[i], HEX);
    }
    Serial.println();

    // Parse if it is a reply from our motor
    if (isReplyFrom(rxId, activeMotorId)) {
      MotorState st;
      if (parseReply(buf, len, st)) {
        Serial.print("  -> angle="); Serial.print(st.angleDeg, 1);
        Serial.print(" deg  speed="); Serial.print(st.speedRPM, 0);
        Serial.print(" RPM  current="); Serial.print(st.currentA, 2);
        Serial.print(" A  temp="); Serial.print(st.tempC);
        Serial.print(" C  err=0x"); Serial.println(st.error, HEX);
      }
    }
  }

  if (Serial.available()) {
    targetAngle = Serial.parseFloat();
    Serial.print("Target: "); Serial.print(targetAngle); Serial.println(" deg");
  }
}
