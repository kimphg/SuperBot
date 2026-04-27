/**
 * MIT Cheetah Mode Support for CubeMars Motors
 *
 * This file contains MIT mode control code that was removed to save memory on Arduino Nano.
 * Use this if you need MIT mode control instead of servo mode.
 *
 * WARNING: Including this will use ~500+ bytes of dynamic memory.
 * Only include if you have memory available.
 *
 * To use: Uncomment the #include in main code and enable MIT mode functions
 */

#ifndef MIT_MODE_ARCHIVE_H
#define MIT_MODE_ARCHIVE_H

// ── MIT Cheetah Mode Configuration ───────────────────────────────────────────
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
// NOTE: Velocity range is FIXED at ±30 rad/s in the CAN message encoding
//       (per official CubeMars sample code), regardless of motor's max velocity
MITConfig mitCfg[2] = {
  { -12.5f, 12.5f, -30.0f,  30.0f, -18.0f, 18.0f, 5.0f, 0.5f },
  { -12.5f, 12.5f, -30.0f,  30.0f, -30.0f, 30.0f, 5.0f, 0.5f },
};

// ── MIT Conversion Function ──────────────────────────────────────────────────
// Matches official CubeMars float_to_uint implementation exactly
static int floatToUintMIT(float x, float xMin, float xMax, uint8_t bits) {
  float span = xMax - xMin;
  if (x < xMin) x = xMin;
  else if (x > xMax) x = xMax;
  return (int)((x - xMin) * ((float)((1 << bits) / span)));
}

// ── MIT Command Functions ────────────────────────────────────────────────────
void sendMITCommand(uint8_t id, uint8_t motorIdx, float posRad, float velRadS, float iff) {
  const MITConfig &cfg = mitCfg[motorIdx];
  int p    = floatToUintMIT(posRad,  cfg.pMin,   cfg.pMax,   16);
  int v    = floatToUintMIT(velRadS, cfg.vMin,   cfg.vMax,   12);
  int kp_  = floatToUintMIT(cfg.kp,  MIT_KP_MIN, MIT_KP_MAX, 12);
  int kd_  = floatToUintMIT(cfg.kd,  MIT_KD_MIN, MIT_KD_MAX, 12);
  int iff_ = floatToUintMIT(iff,     cfg.iffMin, cfg.iffMax, 12);
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
    Serial.print("[MIT CMD RAW] ID="); Serial.print(id); Serial.print(" bytes: ");
    for (uint8_t i = 0; i < 8; i++) {
      if (data[i] < 0x10) Serial.print("0");
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.print(" (p="); Serial.print(p); Serial.print(" v="); Serial.print(v);
    Serial.print(" kp="); Serial.print(kp_); Serial.print(" kd="); Serial.print(kd_);
    Serial.print(" iff="); Serial.print(iff_); Serial.println(")");
  }

  CAN.sendMsgBuf(id, /*ext=*/0, 8, data);
}

// ── MIT Status Reply Parser ──────────────────────────────────────────────────
// MIT reply: can be 6 or 8 bytes (motor sends extended format)
// 6 bytes: [motorId][p_hi][p_lo][v_hi][v_lo|t_hi][t_lo]
// 8 bytes: [motorId][p_hi][p_lo][v_hi][v_lo|kp_hi][kp_lo][kd_hi|iff_hi][iff_lo]
bool parseMITReply(const uint8_t *d, uint8_t len, uint8_t motorIdx, MotorState &s) {
  if (len < 6) return false;
  const MITConfig &cfg = mitCfg[motorIdx];

  // Position is always in bytes 1-2 (same for 6 and 8 byte frames)
  uint16_t p_int = ((uint16_t)d[1] << 8) | d[2];
  float posRad  = p_int / 65535.0f * (cfg.pMax - cfg.pMin) + cfg.pMin;

  // Velocity in bytes 3-4 (bits 0-11 of byte 3 and 4)
  uint16_t v_int = ((uint16_t)d[3] << 4) | (d[4] >> 4);
  float velRadS = v_int / 4095.0f * (cfg.vMax - cfg.vMin) + cfg.vMin;

  // Current/torque
  uint16_t t_int = ((uint16_t)(d[4] & 0xF) << 8) | d[5];
  float currA = t_int / 4095.0f * (cfg.iffMax - cfg.iffMin) + cfg.iffMin;

  s.angleDeg = (int16_t)(posRad * (180.0f / PI));
  s.speedRPM = (int16_t)(velRadS * (60.0f / (2.0f * PI)));
  s.currentA_mA = (int16_t)(currA * 1000.0f);
  s.tempC = 0;
  s.error = 0;
  return true;
}

// ── MIT Mode Switching ───────────────────────────────────────────────────────
void setMITMode(uint8_t id) {
  Serial.print(">> switching motor ID "); Serial.print(id); Serial.println(" to MIT mode");
  uint8_t d[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB };
  uint32_t canId = (uint32_t)id | (0x05UL << 8);
  CAN.sendMsgBuf(canId, /*ext=*/1, 8, d);
  delay(100);
}

#endif // MIT_MODE_ARCHIVE_H
