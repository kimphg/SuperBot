// CubeMars AK48-36 angle control via MCP2515 (SPI CAN) on Teensy 4.0
// MCP2515 CS pin: D10
// Protocol: CubeMars CAN servo mode (position control)
// Library required: mcp_can by coryjfowler (Library Manager: "MCP_CAN")
//
// ── CAN frame layout ──────────────────────────────────────────────────────────
// Command  (host → motor, CAN ID = motorId, 8 bytes):
//   Byte 0:   command type
//               0x0A = position control
//               0x0B = speed control
//               0x0C = torque/current control
//   Byte 1-4: float32 target value (little-endian)
//               position mode: degrees
//               speed mode:    RPM
//               torque mode:   Amps
//   Byte 5-6: int16 speed limit (RPM, little-endian)  [position mode only]
//   Byte 7:   0x00
//
// Reply (motor → host, CAN ID = motorId, 8 bytes):
//   Byte 0:   motorId (echo)
//   Byte 1-2: int16 actual angle  × 10  → divide by 10 for degrees
//   Byte 3-4: int16 actual speed  (RPM)
//   Byte 5-6: int16 actual current (mA)
//   Byte 7:   temperature (°C) or error flags
//
// VERIFY the above against your firmware version / CubeMars manual before use.

#include <SPI.h>
#include <mcp_can.h>

// ── Types ──────────────────────────────────────────────────────────────────────
struct MotorState {
  float angleDeg;   // degrees
  float speedRPM;   // RPM
  float currentA;   // Amps
  uint8_t temp;     // °C
};

// ── MCP2515 ───────────────────────────────────────────────────────────────────
#define CAN_CS_PIN  10
MCP_CAN CAN(CAN_CS_PIN);

// ── Motor enable / disable ────────────────────────────────────────────────────
void motorEnable(uint8_t motorId) {
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  CAN.sendMsgBuf(motorId, 0, 8, data);
}

void motorDisable(uint8_t motorId) {
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  CAN.sendMsgBuf(motorId, 0, 8, data);
}

void motorSetZero(uint8_t motorId) {
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
  CAN.sendMsgBuf(motorId, 0, 8, data);
}

// ── Servo mode helpers ────────────────────────────────────────────────────────
static void packFloat(uint8_t *buf, float v) {
  // Little-endian float32 into buf[0..3]
  memcpy(buf, &v, 4);
}

static void packInt16(uint8_t *buf, int16_t v) {
  buf[0] = (uint8_t)(v & 0xFF);
  buf[1] = (uint8_t)((v >> 8) & 0xFF);
}

// ── Position control command ──────────────────────────────────────────────────
// angleDeg : target angle in degrees
// maxRPM   : speed limit (RPM); 0 = use motor default
void sendPositionCmd(uint8_t motorId, float angleDeg, int16_t maxRPM = 200) {
  uint8_t data[8] = {};
  data[0] = 0x0A;                   // position control command
  packFloat(&data[1], angleDeg);    // bytes 1-4: float32 degrees
  packInt16(&data[5], maxRPM);      // bytes 5-6: speed limit
  data[7] = 0x00;
  CAN.sendMsgBuf(motorId, 0, 8, data);
}

// ── Parse motor reply ─────────────────────────────────────────────────────────
bool parseMotorReply(uint8_t *data, uint8_t len, MotorState &state) {
  if (len < 7) return false;
  int16_t rawAngle   = (int16_t)((uint16_t)data[1] | ((uint16_t)data[2] << 8));
  int16_t rawSpeed   = (int16_t)((uint16_t)data[3] | ((uint16_t)data[4] << 8));
  int16_t rawCurrent = (int16_t)((uint16_t)data[5] | ((uint16_t)data[6] << 8));
  state.angleDeg  = rawAngle  / 10.0f;   // 0.1 deg per count
  state.speedRPM  = (float)rawSpeed;
  state.currentA  = rawCurrent / 1000.0f; // mA → A
  state.temp      = (len >= 8) ? data[7] : 0;
  return true;
}

// ── Motor scan ────────────────────────────────────────────────────────────────
#define SCAN_ID_MIN      1
#define SCAN_ID_MAX      127
#define SCAN_TIMEOUT_MS  100

uint8_t foundIds[SCAN_ID_MAX];
uint8_t foundCount = 0;
static bool idFound[SCAN_ID_MAX + 1];

// Probe one ID: send position query and wait for reply
static bool probeMotor(uint8_t id) {
  // Flush pending frames
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    uint8_t len; uint8_t buf[8]; uint32_t rxId;
    CAN.readMsgBuf(&rxId, &len, buf);
  }

  // Send a zero-degree position command to elicit a reply
  sendPositionCmd(id, 0.0f, 0);

  unsigned long t0 = millis();
  while (millis() - t0 < SCAN_TIMEOUT_MS) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      uint8_t len; uint8_t buf[8]; uint32_t rxId;
      CAN.readMsgBuf(&rxId, &len, buf);
      if (rxId == (uint32_t)id && len >= 6) return true;
    }
  }
  return false;
}

void scanMotors() {
  foundCount = 0;
  memset(idFound, 0, sizeof(idFound));
  Serial.println("Scanning CAN bus for AK48-36 motors...");

  // Phase 1: passive listen — catch motors already broadcasting
  Serial.println("  Phase 1: passive listen (500 ms)...");
  unsigned long t0 = millis();
  while (millis() - t0 < 500) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      uint8_t len; uint8_t buf[8]; uint32_t rxId;
      CAN.readMsgBuf(&rxId, &len, buf);
      if (rxId >= SCAN_ID_MIN && rxId <= SCAN_ID_MAX && len >= 6 && !idFound[rxId]) {
        idFound[rxId] = true;
        foundIds[foundCount++] = (uint8_t)rxId;
        Serial.print("  Found motor ID (passive): ");
        Serial.println((uint8_t)rxId);
      }
    }
  }

  // Phase 2: active probe IDs not yet found
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
uint8_t activeMotorId = 0;

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("MCP2515 init failed, retrying...");
    delay(200);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("MCP2515 ready");

  scanMotors();

  if (foundCount > 0) {
    activeMotorId = foundIds[0];
    Serial.print("Using motor ID: ");
    Serial.println(activeMotorId);
    delay(100);
    motorEnable(activeMotorId);
    Serial.println("Motor enabled (servo mode)");
  }
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  if (activeMotorId == 0) return;

  static float targetAngle = 0.0f;
  static unsigned long lastCmd = 0;

  // 50 Hz command rate
  if (millis() - lastCmd >= 20) {
    lastCmd = millis();
    sendPositionCmd(activeMotorId, targetAngle, /*maxRPM=*/200);
  }

  // Print all incoming CAN frames
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    uint32_t rxId;
    uint8_t  len;
    uint8_t  buf[8];
    CAN.readMsgBuf(&rxId, &len, buf);

    Serial.print("CAN ID=0x"); Serial.print(rxId, HEX);
    Serial.print(" len="); Serial.print(len);
    Serial.print(" data:");
    for (uint8_t i = 0; i < len; i++) {
      Serial.print(" ");
      if (buf[i] < 0x10) Serial.print("0");
      Serial.print(buf[i], HEX);
    }
    Serial.println();

    if (rxId == (uint32_t)activeMotorId) {
      MotorState st;
      if (parseMotorReply(buf, len, st)) {
        Serial.print("  -> angle="); Serial.print(st.angleDeg, 1);
        Serial.print(" deg  speed="); Serial.print(st.speedRPM, 0);
        Serial.print(" RPM  current="); Serial.print(st.currentA, 3);
        Serial.print(" A  temp="); Serial.print(st.temp);
        Serial.println(" C");
      }
    }
  }

  // Accept new target angle from Serial
  if (Serial.available()) {
    targetAngle = Serial.parseFloat();
    Serial.print("Target: "); Serial.print(targetAngle); Serial.println(" deg");
  }
}
