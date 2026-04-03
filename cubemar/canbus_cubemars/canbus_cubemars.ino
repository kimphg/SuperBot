// CubeMars AK48-36 position control via MCP2515 (SPI CAN) on Teensy 4.0
// MCP2515 CS pin: D10
// Library: mcp_can by coryjfowler (Library Manager: "MCP_CAN")
//
// ── Protocol ─────────────────────────────────────────────────────────────────
// Commands  (host → motor): MIT mini-cheetah protocol, STANDARD CAN frame
//   CAN ID = motorId (11-bit)
//   8 bytes, big-endian bit-packed:
//     [15:0]  position  16-bit  P_MIN..P_MAX  (rad)
//     [27:16] velocity  12-bit  V_MIN..V_MAX  (rad/s)
//     [39:28] Kp        12-bit  0..500
//     [51:40] Kd        12-bit  0..5
//     [63:52] torque FF 12-bit  T_MIN..T_MAX  (Nm)
//
// Status reply (motor → host): EXTENDED CAN frame
//   CAN ID = (0x29<<8)|motorId  e.g. 0x296F for ID=111
//   mcp_can sets bit-31 on extended frames → rxId & 0x80000000
//   8 bytes:
//     [0]   status byte
//     [1-2] int16 LE  speed    (0.1 RPM / count) — confirmed from live data
//     [3-4] int16 LE  position (0.1 deg / count) — confirmed: changes with rotation
//     [5]   int8       current  (~1 mA / count, signed; tune scale as needed)
//     [6]   uint8      temperature (°C)
//     [7]   uint8      error (0 = OK)

#include <SPI.h>
#include <mcp_can.h>

// ── MIT protocol limits for AK48-36 ──────────────────────────────────────────
#define P_MIN -12.5f  // rad
#define P_MAX 12.5f
#define V_MIN -30.0f  // rad/s  (covers ~286 RPM output)
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f  // Nm
#define T_MAX 18.0f
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
int float_to_uint(float x, float x_min, float x_max, unsigned int bits)

{

/// Converts a float to an unsigned int, given range and number of bits ///

float span = x_max - x_min;

if(x < x_min) x = x_min;

else if(x > x_max) x = x_max;

return (int) ((x-x_min)*((float)((1<<bits)/span)));

}
void sendCommand(int id, float p_des, float kp = 20, float kd = 1, float t_ff = 10) {
  /// limit data to be within bounds ///
  float v_des = 100;
  // float P_MIN = -95.5;
  // float P_MAX = 95.5;
  // float V_MIN = -30;
  // float V_MAX = 30;
  // float T_MIN = -18;
  // float T_MAX = 18;
  float Kp_MIN = 0;
  float Kp_MAX = 500;
  float Kd_MIN = 0;
  float Kd_MAX = 5;
  float Test_Pos = 0.0;
  p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
  v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
  kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
  kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
  t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
  /// convert floats to unsigned ints ///
  int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
  /// pack ints into the can buffer ///
  canData[0] = p_int >> 8;                             // Position 8 higher
  canData[1] = p_int & 0xFF;                           // Position 8 lower
  canData[2] = v_int >> 4;                             // Speed 8 higher
  canData[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);   //
  canData[4] = kp_int & 0xFF;                          // KP 8 bit lower
  canData[5] = kd_int >> 4;                            // Kd 8 bit higher
  canData[6] = ((kd_int & 0xF) << 4) | (kp_int >> 8);  //KP 4 bit lower torque 4 bit higher
  canData[7] = t_int & 0xff;
  // torque 4 bit lower
  CAN.sendMsgBuf(id, /*ext=*/0, 8, canData);
}
// ── MIT helper: float → unsigned int (n bits) ─────────────────────────────────
static uint16_t floatToUint(float x, float xMin, float xMax, int bits) {
  float span = xMax - xMin;
  float offset = x - xMin;
  if (offset < 0.0f) offset = 0.0f;
  uint32_t maxVal = (1u << bits) - 1;
  uint16_t result = (uint16_t)(offset / span * (float)maxVal);
  if (result > maxVal) result = (uint16_t)maxVal;
  return result;
}

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

// ── MIT command frame ─────────────────────────────────────────────────────────
void sendMotorCommand(uint8_t id,
                      float posRad, float velRad_s,
                      float kp, float kd,
                      float torqueFF_Nm) {
  uint16_t p = floatToUint(posRad, P_MIN, P_MAX, 16);
  uint16_t v = floatToUint(velRad_s, V_MIN, V_MAX, 12);
  uint16_t kpI = floatToUint(kp, KP_MIN, KP_MAX, 12);
  uint16_t kdI = floatToUint(kd, KD_MIN, KD_MAX, 12);
  uint16_t t = floatToUint(torqueFF_Nm, T_MIN, T_MAX, 12);

  uint8_t d[8];
  d[0] = (uint8_t)(p >> 8);
  d[1] = (uint8_t)(p & 0xFF);
  d[2] = (uint8_t)(v >> 4);
  d[3] = (uint8_t)((v & 0x0F) << 4) | (uint8_t)(kpI >> 8);
  d[4] = (uint8_t)(kpI & 0xFF);
  d[5] = (uint8_t)(kdI >> 4);
  d[6] = (uint8_t)((kdI & 0x0F) << 4) | (uint8_t)(t >> 8);
  d[7] = (uint8_t)(t & 0xFF);

  CAN.sendMsgBuf(id, /*ext=*/0, 8, d);
}

// Position servo: degrees → MIT command (no vel/torque feedforward)
void sendAngleControl(uint8_t id, float angleDeg, float kp, float kd) {
  float angleRad = angleDeg * (float)M_PI / 180.0f;
  // sendMotorCommand(id, angleRad, 0.0f, kp, kd, 0.0f);
  sendCommand(id, angleRad, kp, kd, 10);
}

// ── Status reply parser ───────────────────────────────────────────────────────
// Motor sends extended frame; mcp_can marks bit-31 of rxId for extended frames.
// Motor ID is in the low byte of the actual 29-bit CAN ID.
static bool isReplyFrom(uint32_t rxId, uint8_t motorId) {
  return (rxId & 0x80000000) && ((rxId & 0xFF) == motorId);
}

bool parseReply(const uint8_t *d, uint8_t len, MotorState &s) {
  if (len < 7) return false;
  int16_t rawPos = (int16_t)((uint16_t)d[1] | ((uint16_t)d[0] << 8));
  int16_t rawSpeed = (int16_t)(((uint16_t)d[3] >> 4) | ((uint16_t)d[2] << 4));
  s.speedRPM = rawSpeed * 0.1f;        // 0.1 RPM per count
  s.angleDeg = rawPos * 0.1f;          // 0.1 deg per count
  s.currentA = (int8_t)d[5] * 0.001f;  // ~1 mA per count (tune if needed)
  s.tempC = d[6];
  s.error = (len >= 8) ? d[7] : 0;
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

  motorEnable(id);  // MIT enter command → motor replies with status frame

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
uint8_t activeMotorId = 0;
float initialAngle = 0.0f;

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
    Serial.print("Using motor ID: ");
    Serial.println(activeMotorId);
    delay(100);
    motorEnable(activeMotorId);
    Serial.println("Motor enabled");

    // Read initial position so motor holds in place on startup
    delay(50);
    unsigned long t0 = millis();
    while (millis() - t0 < 200) {
      if (CAN.checkReceive() == CAN_MSGAVAIL) {
        uint32_t rxId;
        uint8_t len;
        uint8_t buf[8];
        CAN.readMsgBuf(&rxId, &len, buf);
        if (isReplyFrom(rxId, activeMotorId) && len >= 5) {
          int16_t rawPos = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
          initialAngle = rawPos * 0.1f;
          Serial.print("Initial angle: ");
          Serial.print(initialAngle);
          Serial.println(" deg");
          break;
        }
      }
    }
  }
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  if (activeMotorId == 0) return;

  static float targetAngle = -1.0e10f;
  if (targetAngle < -1.0e9f) targetAngle = initialAngle;

  static unsigned long lastCmd = 0;
  if (millis() - lastCmd >= 20) {  // 50 Hz
    lastCmd = millis();
    sendAngleControl(activeMotorId, targetAngle, /*Kp=*/50.0f, /*Kd=*/1.0f);
  }

  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    uint32_t rxId;
    uint8_t len;
    uint8_t buf[8];
    CAN.readMsgBuf(&rxId, &len, buf);

    Serial.print("CAN ID=0x");
    Serial.print(rxId, HEX);
    Serial.print(rxId & 0x80000000 ? " [ext]" : " [std]");
    Serial.print(" len=");
    Serial.print(len);
    Serial.print(" data:");
    for (uint8_t i = 0; i < len; i++) {
      Serial.print(" ");
      if (buf[i] < 0x10) Serial.print("0");
      Serial.print(buf[i], HEX);
    }
    Serial.println();

    if (isReplyFrom(rxId, activeMotorId)) {
      MotorState st;
      if (parseReply(buf, len, st)) {
        Serial.print("  -> angle=");
        Serial.print(st.angleDeg, 1);
        Serial.print(" deg  speed=");
        Serial.print(st.speedRPM, 1);
        Serial.print(" RPM  current=");
        Serial.print(st.currentA * 1000.0f, 0);
        Serial.print(" mA  temp=");
        Serial.print(st.tempC);
        Serial.print(" C  err=0x");
        Serial.println(st.error, HEX);
      }
    }
  }

  if (Serial.available()) {
    float val = Serial.parseFloat();
    while (Serial.available()) Serial.read();  // discard leftover newline/whitespace
    targetAngle = val;
    Serial.print("Target: ");
    Serial.print(targetAngle);
    Serial.println(" deg");
  }
}
