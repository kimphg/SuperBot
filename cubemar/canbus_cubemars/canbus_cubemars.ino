// CubeMars AK60 angle control via MCP2515 (SPI CAN) on Teensy 4.0
// MCP2515 CS pin: D10
// Protocol: MIT mini-cheetah motor protocol
// Library required: mcp_can by coryjfowler (Library Manager: "MCP_CAN")

#include <SPI.h>
#include <mcp_can.h>

// ── Types (must appear before Arduino auto-generated prototypes) ──────────────
struct MotorState { float pos, vel, torque; };

// ── MCP2515 ──────────────────────────────────────────────────────────────────
#define CAN_CS_PIN  10
MCP_CAN CAN(CAN_CS_PIN);

// ── AK60 motor limits (MIT protocol) ─────────────────────────────────────────
#define P_MIN   -12.5f   // rad
#define P_MAX    12.5f
#define V_MIN   -45.0f   // rad/s
#define V_MAX    45.0f
#define KP_MIN    0.0f
#define KP_MAX  500.0f
#define KD_MIN    0.0f
#define KD_MAX    5.0f
#define T_MIN   -15.0f   // Nm
#define T_MAX    15.0f

// ── Helper: float → unsigned int (bit-width n) ───────────────────────────────
static uint16_t floatToUint(float x, float xMin, float xMax, int bits) {
  float span = xMax - xMin;
  float offset = x - xMin;
  if (offset < 0.0f) offset = 0.0f;
  uint32_t maxVal = (1u << bits) - 1;
  uint16_t result = (uint16_t)(offset / span * (float)maxVal);
  if (result > maxVal) result = (uint16_t)maxVal;
  return result;
}

// ── Special commands ──────────────────────────────────────────────────────────
void motorEnter(uint8_t motorId) {
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  CAN.sendMsgBuf(motorId, 0, 8, data);
}

void motorExit(uint8_t motorId) {
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  CAN.sendMsgBuf(motorId, 0, 8, data);
}

void motorZero(uint8_t motorId) {
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
  CAN.sendMsgBuf(motorId, 0, 8, data);
}

// ── MIT protocol frame ────────────────────────────────────────────────────────
// Byte layout (64 bits total):
//   [15:0]  position   16 bits
//   [27:16] velocity   12 bits
//   [39:28] Kp         12 bits
//   [51:40] Kd         12 bits
//   [63:52] torque ff  12 bits
void sendMotorCommand(uint8_t motorId,
                      float posRad,
                      float velRad_s,
                      float kp,
                      float kd,
                      float torqueFF_Nm) {
  uint16_t p   = floatToUint(posRad,      P_MIN,  P_MAX,  16);
  uint16_t v   = floatToUint(velRad_s,    V_MIN,  V_MAX,  12);
  uint16_t kpI = floatToUint(kp,          KP_MIN, KP_MAX, 12);
  uint16_t kdI = floatToUint(kd,          KD_MIN, KD_MAX, 12);
  uint16_t t   = floatToUint(torqueFF_Nm, T_MIN,  T_MAX,  12);

  uint8_t data[8];
  data[0] = (uint8_t)(p >> 8);
  data[1] = (uint8_t)(p & 0xFF);
  data[2] = (uint8_t)(v >> 4);
  data[3] = (uint8_t)((v & 0x0F) << 4) | (uint8_t)(kpI >> 8);
  data[4] = (uint8_t)(kpI & 0xFF);
  data[5] = (uint8_t)(kdI >> 4);
  data[6] = (uint8_t)((kdI & 0x0F) << 4) | (uint8_t)(t >> 8);
  data[7] = (uint8_t)(t & 0xFF);

  CAN.sendMsgBuf(motorId, 0, 8, data);
}

// ── Angle control convenience wrapper ────────────────────────────────────────
// Sends a position command with PD gains; no velocity or torque feedforward.
void sendAngleControl(uint8_t motorId, float angleDeg, float kp, float kd) {
  float angleRad = angleDeg * (float)M_PI / 180.0f;
  sendMotorCommand(motorId, angleRad, 0.0f, kp, kd, 0.0f);
}

// ── Parse motor reply ─────────────────────────────────────────────────────────
// Reply: 6 bytes, CAN ID = motorId
//   [14:0]  position  15 bits  → -4π..4π rad
//   [26:15] velocity  12 bits  → -45..45 rad/s
//   [38:27] torque    12 bits  → -15..15 Nm
static float uintToFloat(uint16_t x, float xMin, float xMax, int bits) {
  float span = xMax - xMin;
  return (float)x / (float)((1u << bits) - 1) * span + xMin;
}

bool readMotorReply(uint8_t motorId, MotorState &state) {
  if (CAN.checkReceive() != CAN_MSGAVAIL) return false;

  uint8_t len;
  uint8_t data[8];
  uint32_t rxId;
  CAN.readMsgBuf(&rxId, &len, data);

  if (rxId != (uint32_t)(motorId) || len < 6) return false;

  uint16_t p = ((uint16_t)(data[1] & 0xFF) << 7) | (data[2] >> 1);
  uint16_t v = ((uint16_t)(data[2] & 0x01) << 11) | ((uint16_t)data[3] << 3) | (data[4] >> 5);
  uint16_t t = ((uint16_t)(data[4] & 0x1F) << 7) | (data[5] >> 1);

  state.pos    = uintToFloat(p, -4.0f * (float)M_PI, 4.0f * (float)M_PI, 15);
  state.vel    = uintToFloat(v, V_MIN, V_MAX, 12);
  state.torque = uintToFloat(t, T_MIN, T_MAX, 12);
  return true;
}

// ── Motor scan ────────────────────────────────────────────────────────────────
#define SCAN_ID_MIN       1
#define SCAN_ID_MAX       127
#define SCAN_TIMEOUT_MS   100  // wait per ID for a reply (was 10 — too short)

uint8_t foundIds[SCAN_ID_MAX];
uint8_t foundCount = 0;

// Returns true if motorId replies within SCAN_TIMEOUT_MS after motorEnter.
// Sends motorExit afterward so the motor stays idle until explicitly used.
static bool probMotor(uint8_t id) {
  // Flush any pending frames
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    uint8_t len; uint8_t buf[8]; uint32_t rxId;
    CAN.readMsgBuf(&rxId, &len, buf);
  }

  motorEnter(id);

  unsigned long t0 = millis();
  while (millis() - t0 < SCAN_TIMEOUT_MS) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      uint8_t len; uint8_t buf[8]; uint32_t rxId;
      CAN.readMsgBuf(&rxId, &len, buf);
      if (rxId == (uint32_t)id && len >= 6) {
        motorExit(id);
        return true;
      }
    }
  }
  return false;
}

// Track which IDs were already found to avoid duplicates
static bool idFound[SCAN_ID_MAX + 1];

void scanMotors() {
  foundCount = 0;
  memset(idFound, 0, sizeof(idFound));
  Serial.println("Scanning CAN bus for AK60 motors...");

  // Phase 1: passive listen — catch motors broadcasting spontaneously
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

  // Phase 2: active probe — send motorEnter to each ID not already found
  Serial.println("  Phase 2: active probe...");
  for (uint8_t id = SCAN_ID_MIN; id <= SCAN_ID_MAX; id++) {
    if (idFound[id]) continue;
    if (probMotor(id)) {
      idFound[id] = true;
      foundIds[foundCount++] = id;
      Serial.print("  Found motor ID (active): ");
      Serial.println(id);
    }
  }
  if (foundCount == 0) {
    Serial.println("No motors found.");
    delay(2000);scanMotors();
  } else {
    Serial.print(foundCount);
    Serial.println(" motor(s) found.");
    
  }
}

// ── Runtime motor ID (first found, or 0 if none) ──────────────────────────────
uint8_t activeMotorId = 0;

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
    motorEnter(activeMotorId);
    Serial.println("Motor enabled");
  }
}

void loop() {
  if (activeMotorId == 0) return;   // no motor found

  static float angle = 0.0f;
  static unsigned long lastCmd = 0;

  if (millis() - lastCmd >= 20) {   // 50 Hz command rate
    lastCmd = millis();
    sendAngleControl(activeMotorId, angle, /*Kp=*/50.0f, /*Kd=*/0.5f);
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

    // Parse as motor reply if ID matches active motor
    if (rxId == (uint32_t)activeMotorId && len >= 6) {
      uint16_t p = ((uint16_t)(buf[1] & 0xFF) << 7) | (buf[2] >> 1);
      uint16_t v = ((uint16_t)(buf[2] & 0x01) << 11) | ((uint16_t)buf[3] << 3) | (buf[4] >> 5);
      uint16_t t = ((uint16_t)(buf[4] & 0x1F) << 7) | (buf[5] >> 1);
      MotorState st;
      st.pos    = uintToFloat(p, -4.0f * (float)M_PI, 4.0f * (float)M_PI, 15);
      st.vel    = uintToFloat(v, V_MIN, V_MAX, 12);
      st.torque = uintToFloat(t, T_MIN, T_MAX, 12);
      Serial.print("  -> pos="); Serial.print(st.pos * 180.0f / M_PI, 2);
      Serial.print(" deg  vel="); Serial.print(st.vel, 2);
      Serial.print(" rad/s  torque="); Serial.print(st.torque, 2);
      Serial.println(" Nm");
    }
  }

  if (Serial.available()) {
    angle = Serial.parseFloat();
    Serial.print("Target: "); Serial.print(angle); Serial.println(" deg");
  }
}
