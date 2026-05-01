/*
 * ESP32 LoRa MAVLink GPS Receiver
 *
 * Listens on the same LoRa radio settings as esp32-lora-sensor, validates the
 * MAVLink v1 framing + CRC, and decodes GPS_RAW_INT (msg id 24) packets to
 * the USB serial console.
 */

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// LoRa pins — match the transmitter (Heltec / TTGO style ESP32-C3 boards).
#define LORA_SCK   4
#define LORA_MISO  5
#define LORA_MOSI  6
#define LORA_SS    7
#define LORA_RST   3
#define LORA_DI0   2

// LoRa radio parameters — must match the sensor exactly. Both ends use the
// Ai-Thinker RA-02 module (SX1278, 433 MHz antenna match).
#define LORA_FREQUENCY         433E6
#define LORA_SPREADING_FACTOR  7
#define LORA_SIGNAL_BANDWIDTH  125E3
#define LORA_CODING_RATE       5
#define LORA_PREAMBLE_LENGTH   8
#define LORA_SYNC_WORD         0x12

// MAVLink v1 frame layout: 0xFE [len] [seq] [sysid] [compid] [msgid] [payload] [crc_lo] [crc_hi]
#define MAV_V1_MAGIC           0xFE
#define MAV_HEADER_LEN         6
#define MAV_FOOTER_LEN         2
#define MAV_MAX_PAYLOAD        255

#define MSG_ID_GPS_RAW_INT     24
#define CRC_EXTRA_GPS_RAW_INT  24
#define GPS_RAW_INT_LEN        30

void initLoRa();
void handlePacket(int packetSize);
bool validateMavV1(const uint8_t *buf, int len);
void decodeGpsRawInt(const uint8_t *payload, uint8_t sysid, uint8_t compid, int rssi, float snr);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\nESP32 LoRa MAVLink Receiver Starting...");
  Serial.println("=========================================");

  initLoRa();

  Serial.println("Listening for MAVLink GPS packets...\n");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    handlePacket(packetSize);
  }
}

void initLoRa() {
  Serial.println("Initializing LoRa radio...");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DI0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed!");
    while (true) delay(1000);
  }

  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
  LoRa.setSyncWord(LORA_SYNC_WORD);

  Serial.println("LoRa initialization complete");
}

void handlePacket(int packetSize) {
  if (packetSize < MAV_HEADER_LEN + MAV_FOOTER_LEN) return;
  if (packetSize > MAV_HEADER_LEN + MAV_MAX_PAYLOAD + MAV_FOOTER_LEN) return;

  uint8_t buf[MAV_HEADER_LEN + MAV_MAX_PAYLOAD + MAV_FOOTER_LEN];
  int n = 0;
  while (LoRa.available() && n < (int)sizeof(buf)) {
    buf[n++] = (uint8_t)LoRa.read();
  }
  if (n != packetSize) return;

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  if (!validateMavV1(buf, n)) {
    Serial.print("Bad MAVLink frame  rssi="); Serial.print(rssi);
    Serial.print(" snr="); Serial.print(snr, 1);
    Serial.print(" bytes="); Serial.println(n);
    return;
  }

  uint8_t sysid  = buf[3];
  uint8_t compid = buf[4];
  uint8_t msgid  = buf[5];
  const uint8_t *payload = &buf[MAV_HEADER_LEN];

  if (msgid == MSG_ID_GPS_RAW_INT) {
    decodeGpsRawInt(payload, sysid, compid, rssi, snr);
  } else {
    Serial.print("Unhandled MAVLink msgid=");
    Serial.print(msgid);
    Serial.print("  sysid="); Serial.print(sysid);
    Serial.print(" compid="); Serial.println(compid);
  }
}

// MAVLink CRC accumulator (X.25 / CCITT, polynomial 0x1021 reflected).
static inline void mav_crc_accumulate(uint8_t data, uint16_t *crc) {
  uint8_t tmp = data ^ (uint8_t)(*crc & 0xff);
  tmp ^= (tmp << 4);
  *crc = (*crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4);
}

// Returns the CRC_EXTRA byte for known message IDs, or 0xFF if unsupported.
static uint8_t crcExtraFor(uint8_t msgid) {
  switch (msgid) {
    case MSG_ID_GPS_RAW_INT: return CRC_EXTRA_GPS_RAW_INT;
    default: return 0xFF;
  }
}

bool validateMavV1(const uint8_t *buf, int len) {
  if (buf[0] != MAV_V1_MAGIC) return false;
  uint8_t payloadLen = buf[1];
  if (len != MAV_HEADER_LEN + payloadLen + MAV_FOOTER_LEN) return false;

  uint8_t msgid = buf[5];
  uint8_t crcExtra = crcExtraFor(msgid);
  if (crcExtra == 0xFF) return false;  // unknown msg, can't verify CRC

  // Sanity: GPS_RAW_INT must be 30 bytes.
  if (msgid == MSG_ID_GPS_RAW_INT && payloadLen != GPS_RAW_INT_LEN) return false;

  uint16_t crc = 0xFFFF;
  for (int i = 1; i < MAV_HEADER_LEN + payloadLen; i++) {
    mav_crc_accumulate(buf[i], &crc);
  }
  mav_crc_accumulate(crcExtra, &crc);

  uint16_t rxCrc = (uint16_t)buf[MAV_HEADER_LEN + payloadLen]
                 | ((uint16_t)buf[MAV_HEADER_LEN + payloadLen + 1] << 8);
  return crc == rxCrc;
}

// Little-endian readers for the MAVLink wire format.
static inline uint16_t rd_u16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline uint32_t rd_u32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline uint64_t rd_u64(const uint8_t *p) {
  uint64_t v = 0;
  for (int i = 0; i < 8; i++) v |= ((uint64_t)p[i]) << (8 * i);
  return v;
}

static const char *fixTypeName(uint8_t f) {
  switch (f) {
    case 0: return "NO_GPS";
    case 1: return "NO_FIX";
    case 2: return "2D";
    case 3: return "3D";
    case 4: return "DGPS";
    case 5: return "RTK_FLOAT";
    case 6: return "RTK_FIXED";
    case 7: return "STATIC";
    case 8: return "PPP";
    default: return "?";
  }
}

void decodeGpsRawInt(const uint8_t *p, uint8_t sysid, uint8_t compid, int rssi, float snr) {
  uint64_t time_us = rd_u64(p);          p += 8;
  int32_t  lat_e7  = (int32_t)rd_u32(p); p += 4;
  int32_t  lon_e7  = (int32_t)rd_u32(p); p += 4;
  int32_t  alt_mm  = (int32_t)rd_u32(p); p += 4;
  uint16_t eph     = rd_u16(p);          p += 2;
  uint16_t epv     = rd_u16(p);          p += 2;
  uint16_t vel_cms = rd_u16(p);          p += 2;
  uint16_t cog_cd  = rd_u16(p);          p += 2;
  uint8_t  fixType = *p++;
  uint8_t  sats    = *p++;
  (void)epv;

  double lat = lat_e7 / 1e7;
  double lon = lon_e7 / 1e7;
  double alt = alt_mm / 1000.0;
  double hdop = eph / 100.0;
  double speed_ms = vel_cms / 100.0;
  double cog_deg = cog_cd / 100.0;

  Serial.println("--- GPS_RAW_INT ---");
  Serial.print("  link    rssi="); Serial.print(rssi);
  Serial.print(" dBm  snr="); Serial.print(snr, 1); Serial.println(" dB");
  Serial.print("  from    sys="); Serial.print(sysid);
  Serial.print(" comp=");        Serial.println(compid);
  Serial.print("  fix     ");    Serial.print(fixType);
  Serial.print(" (");            Serial.print(fixTypeName(fixType));
  Serial.print(")  sats=");      Serial.println(sats);
  Serial.print("  lat     ");    Serial.println(lat, 7);
  Serial.print("  lon     ");    Serial.println(lon, 7);
  Serial.print("  alt     ");    Serial.print(alt, 2);   Serial.println(" m");
  Serial.print("  hdop    ");    Serial.println(hdop, 2);
  Serial.print("  speed   ");    Serial.print(speed_ms, 2); Serial.println(" m/s");
  Serial.print("  course  ");    Serial.print(cog_deg, 1);  Serial.println(" deg");
  Serial.print("  time_us ");    Serial.println((unsigned long long)time_us);
  Serial.println();
}
