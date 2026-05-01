/*
 * ESP32 LoRa Sensor Node
 * 
 * A sensor node that reads data and transmits via LoRa radio.
 * Compile with: arduino-cli compile -b esp32:esp32:esp32
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

// LoRa pins for common ESP32 LoRa boards (Heltec, TTGO, etc.)
#define LORA_SCK 4
#define LORA_MISO 5
#define LORA_MOSI 6
#define LORA_SS 7
#define LORA_RST 3
#define LORA_DI0 2

// I2C configuration for MS5611 barometric pressure sensor (GY-63 / GY-86)
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_FREQUENCY 100000  // 100kHz

// Secondary Serial (Debug/External) on D7(RX)/D6(Tx)
// Use Serial1 as Serial2 may not be available on all ESP32 variants
#define SERIAL1_RX 20
#define SERIAL1_TX 21
#define SERIAL1_BAUD 9600

// MS5611 — I2C address: 0x77 when CSB is tied low (default on most breakouts),
// 0x76 when CSB is pulled high. The MS5611 uses commands instead of register
// addresses for data: write a 1-byte command, then for ADC reads start a new
// transaction reading 3 bytes; for PROM reads, read 2 bytes.
#define MS5611_ADDR             0x77

#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_CONVERT_D1   0x40   // pressure, OR'd with OSR
#define MS5611_CMD_CONVERT_D2   0x50   // temperature, OR'd with OSR
#define MS5611_CMD_ADC_READ     0x00
#define MS5611_CMD_PROM_READ    0xA0   // C1..C6 live at 0xA2,0xA4,...0xAC

// OSR (oversampling): 0=256, 2=512, 4=1024, 6=2048, 8=4096. Higher = quieter
// but slower. 4096 takes ~9 ms per conversion (10 ms with margin).
#define MS5611_OSR              0x08
#define MS5611_CONV_DELAY_MS    10

// Many "MS5611" boards actually carry an MS5607 die — same I2C layout, but
// the pressure compensation uses shift counts one larger, so MS5611 math on
// an MS5607 gives ~half the real pressure (and an altitude reading thousands
// of metres too high). Set this to 0 if your chip is a genuine MS5611.
#define MS56XX_IS_MS5607        1

#if MS56XX_IS_MS5607
  #define MS56XX_OFF_C2_SHIFT    17
  #define MS56XX_OFF_C4_SHIFT     6
  #define MS56XX_SENS_C1_SHIFT   16
  #define MS56XX_SENS_C3_SHIFT    7
#else
  #define MS56XX_OFF_C2_SHIFT    16
  #define MS56XX_OFF_C4_SHIFT     7
  #define MS56XX_SENS_C1_SHIFT   15
  #define MS56XX_SENS_C3_SHIFT    8
#endif

// Standard sea-level pressure for the altitude formula.
#define SEA_LEVEL_PRESSURE_PA   101325.0f

// PROM calibration constants C1..C6 (index 0 unused).
uint16_t ms5611_c[7];

// LoRa configuration — Ai-Thinker RA-02 module (SX1278, 410-525 MHz band).
// Antenna matching is tuned for 433 MHz; do not use the US 915 MHz setting.
#define LORA_FREQUENCY 433E6  // 433 MHz ISM band
#define LORA_TX_POWER 17      // Transmit power in dBm (PA_BOOST output, RA-02)
#define LORA_SPREADING_FACTOR 7
#define LORA_SIGNAL_BANDWIDTH 125E3
#define LORA_CODING_RATE 5    // 4/5 coding rate
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYNC_WORD 0x12

// Sensor reading interval (milliseconds)
#define SENSOR_READ_INTERVAL 5000  // 5 seconds for debugging

// LED pin
#define LED_PIN 2

// MAVLink identity
#define MAV_SYSID    1
#define MAV_COMPID   220   // MAV_COMP_ID_GPS

// Latest GPS state, accumulated from NMEA sentences and emitted as GPS_RAW_INT.
struct GpsState {
  bool      have_time;     // unix time was parsed from RMC
  uint64_t  time_unix_us;  // unix epoch microseconds
  int32_t   lat_e7;        // 1E-7 degrees
  int32_t   lon_e7;        // 1E-7 degrees
  int32_t   alt_mm;        // mm above MSL (from GGA)
  uint16_t  eph;           // HDOP * 100
  uint16_t  vel_cms;       // ground speed cm/s
  uint16_t  cog_cdeg;      // course over ground centi-deg
  uint8_t   fix_type;      // MAVLink GPS_FIX_TYPE
  uint8_t   sats;          // satellites visible
} g_gps;

uint8_t g_mav_seq = 0;

void initLoRa();
void initI2C();
void scanI2C();
void initSerial1();
bool initMS5611();
void readMS5611(float &temp_c, float &pressure_pa, float &altitude_m);
void sendSensorData();
void handleNmeaLine(const String &line);
void parseRMC(const String &line);
void parseGGA(const String &line);
void sendGpsRawInt();

void setup() {
  // Initialize LED first to give visual feedback
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // LED on during init
  
  Serial.begin(115200);
  delay(500);  // Wait for serial to stabilize
  
  Serial.println("\n\nESP32 LoRa Sensor Node Starting...");
  Serial.println("=====================================");
  
  // Initialize I2C and MS5611 barometer
  initI2C();
  scanI2C();

  // Initialize secondary serial
  initSerial1();
  delay(100);

  if (initMS5611()) {
    Serial.println("MS5611 sensor found and initialized");
  } else {
    Serial.println("Warning: MS5611 sensor not found!");
  }
  
  // Initialize LoRa
  initLoRa();

  digitalWrite(LED_PIN, LOW);

  Serial.println("LoRa Sensor Node Ready!");
  Serial.println("=====================================\n");
}

void loop() {
  static String nmeaLine;

  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n') {
      nmeaLine.trim();
      if (nmeaLine.startsWith("$")) {
        handleNmeaLine(nmeaLine);
      }
      nmeaLine = "";
    } else if (c != '\r') {
      nmeaLine += c;
      if (nmeaLine.length() > 120) {  // NMEA max is 82; guard against runaway
        nmeaLine = "";
      }
    }
  }

  // Periodically read and print MS5611 sensor data (non-blocking via millis()).
  // Initial value ensures the first reading fires on the very first loop pass.
  static uint32_t lastBaroMs = (uint32_t)-SENSOR_READ_INTERVAL;
  if (millis() - lastBaroMs >= SENSOR_READ_INTERVAL) {
    lastBaroMs = millis();
    sendSensorData();
  }
}

// Direct SPI read of an SX127x register, bypassing the LoRa library — used to
// confirm we can actually talk to the chip before LoRa.begin() runs.
static uint8_t loraSpiRead(uint8_t reg) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(LORA_SS, LOW);
  SPI.transfer(reg & 0x7F);          // top bit 0 = read
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(LORA_SS, HIGH);
  SPI.endTransaction();
  return value;
}

void initLoRa() {
  Serial.println("Initializing LoRa radio...");
  Serial.print("  pins: SCK=");  Serial.print(LORA_SCK);
  Serial.print(" MISO=");        Serial.print(LORA_MISO);
  Serial.print(" MOSI=");        Serial.print(LORA_MOSI);
  Serial.print(" SS=");          Serial.print(LORA_SS);
  Serial.print(" RST=");         Serial.print(LORA_RST);
  Serial.print(" DIO0=");        Serial.println(LORA_DI0);

  // Drive SS high (deselected) and pulse RESET — some SX127x clones need a
  // longer reset window than the LoRa library provides.
  pinMode(LORA_SS, OUTPUT);
  digitalWrite(LORA_SS, HIGH);
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);
  delay(20);
  digitalWrite(LORA_RST, HIGH);
  delay(50);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

  // Probe RegVersion (0x42). Read it 3× to check stability — a floating MISO
  // pin will give different bytes every read.
  uint8_t v1 = loraSpiRead(0x42);
  uint8_t v2 = loraSpiRead(0x42);
  uint8_t v3 = loraSpiRead(0x42);
  Serial.print("  RegVersion (0x42) reads: 0x");
  Serial.print(v1, HEX); Serial.print(" 0x");
  Serial.print(v2, HEX); Serial.print(" 0x");
  Serial.println(v3, HEX);

  if (v1 != v2 || v2 != v3) {
    Serial.println("  -> Inconsistent reads: MISO likely floating or noisy.");
    Serial.println("     Check MISO wiring and that the module has 3.3V power.");
  } else if (v1 == 0x00 || v1 == 0xFF) {
    Serial.println("  -> No SPI response from radio.");
    Serial.println("     Check: VCC=3.3V, GND, SS/SCK/MOSI/MISO/RST wiring,");
    Serial.println("     and that the module is actually seated.");
  } else if (v1 == 0x12) {
    Serial.println("  -> SX1276/77/78/79 detected (chip is alive)");
  } else if (v1 == 0x22) {
    Serial.println("  -> SX1272/73 detected");
  } else {
    Serial.print("  -> Unknown chip ID 0x"); Serial.println(v1, HEX);
  }

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DI0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed!");
    return;
  }

  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
  LoRa.setSyncWord(LORA_SYNC_WORD);

  Serial.println("LoRa initialization complete");
}

void initI2C() {
  Serial.println("Initializing I2C...");
  Serial.print("SDA Pin: "); Serial.println(I2C_SDA);
  Serial.print("SCL Pin: "); Serial.println(I2C_SCL);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQUENCY);
  Serial.println("I2C initialized");
}

void initSerial1() {
  Serial1.begin(SERIAL1_BAUD, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
  Serial.print("Serial1 initialized on RX=");
  Serial.print(SERIAL1_RX);
  Serial.print(" TX=");
  Serial.println(SERIAL1_TX);
  Serial1.println("\n[Serial1] ESP32 Sensor Node Ready");
}

void scanI2C() {
  Serial.println("\n=== I2C Scan ===");
  Serial.println("Scanning for I2C devices...");
  
  int devicesFound = 0;
  
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("  Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      devicesFound++;
    }
  }
  
  if (devicesFound == 0) {
    Serial.println("  No devices found!");
  } else {
    Serial.print("  Total devices: ");
    Serial.println(devicesFound);
  }
  Serial.println("===============\n");
}

// Read the 24-bit ADC result of the last conversion (D1 or D2).
static uint32_t ms5611_readAdc() {
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(MS5611_CMD_ADC_READ);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom((uint8_t)MS5611_ADDR, (uint8_t)3);
  if (Wire.available() < 3) return 0;
  uint32_t v = (uint32_t)Wire.read() << 16;
  v |= (uint32_t)Wire.read() << 8;
  v |= (uint32_t)Wire.read();
  return v;
}

bool initMS5611() {
  // Issue RESET (reloads PROM coefficients into the chip)
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(MS5611_CMD_RESET);
  if (Wire.endTransmission() != 0) {
    Serial.println("Error: No response from MS5611");
    return false;
  }
  delay(3);  // PROM reload takes ~2.8 ms

  // Read calibration words C1..C6
  for (uint8_t i = 1; i <= 6; i++) {
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(MS5611_CMD_PROM_READ + (i * 2));
    if (Wire.endTransmission(false) != 0) {
      Serial.println("Error: MS5611 PROM addr write failed");
      return false;
    }
    Wire.requestFrom((uint8_t)MS5611_ADDR, (uint8_t)2);
    if (Wire.available() < 2) {
      Serial.println("Error: MS5611 PROM read timeout");
      return false;
    }
    ms5611_c[i] = ((uint16_t)Wire.read() << 8) | (uint16_t)Wire.read();
  }

  // A useful sanity check: a freshly-blanked / disconnected MS5611 returns
  // either all-zero or all-0xFFFF for every PROM word.
  if ((ms5611_c[1] == 0 && ms5611_c[2] == 0) ||
      (ms5611_c[1] == 0xFFFF && ms5611_c[2] == 0xFFFF)) {
    Serial.println("Error: MS5611 PROM appears empty");
    return false;
  }

  Serial.print("MS5611 PROM:");
  for (int i = 1; i <= 6; i++) {
    Serial.print(" C"); Serial.print(i);
    Serial.print("="); Serial.print(ms5611_c[i]);
  }
  Serial.println();

  return true;
}

// Trigger a D1 (pressure) and D2 (temperature) conversion, read both, and run
// the datasheet-prescribed compensation (with the second-order low-temperature
// correction). Output is °C, Pa, and metres above MSL.
void readMS5611(float &temp_c, float &pressure_pa, float &altitude_m) {
  // --- D2: temperature ---
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(MS5611_CMD_CONVERT_D2 | MS5611_OSR);
  if (Wire.endTransmission() != 0) {
    temp_c = pressure_pa = altitude_m = 0;
    Serial.println("Error: MS5611 D2 trigger failed");
    return;
  }
  delay(MS5611_CONV_DELAY_MS);
  uint32_t D2 = ms5611_readAdc();

  // --- D1: pressure ---
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(MS5611_CMD_CONVERT_D1 | MS5611_OSR);
  if (Wire.endTransmission() != 0) {
    temp_c = pressure_pa = altitude_m = 0;
    Serial.println("Error: MS5611 D1 trigger failed");
    return;
  }
  delay(MS5611_CONV_DELAY_MS);
  uint32_t D1 = ms5611_readAdc();

  if (D1 == 0 || D2 == 0) {
    temp_c = pressure_pa = altitude_m = 0;
    Serial.println("Error: MS5611 ADC read failed");
    return;
  }

  // First-order compensation (per MS5611 / MS5607 datasheet).
  int32_t dT   = (int32_t)D2 - ((int32_t)ms5611_c[5] << 8);
  int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * ms5611_c[6]) >> 23);
  int64_t OFF  = ((int64_t)ms5611_c[2] << MS56XX_OFF_C2_SHIFT)
               + (((int64_t)ms5611_c[4] * dT) >> MS56XX_OFF_C4_SHIFT);
  int64_t SENS = ((int64_t)ms5611_c[1] << MS56XX_SENS_C1_SHIFT)
               + (((int64_t)ms5611_c[3] * dT) >> MS56XX_SENS_C3_SHIFT);

  // Second-order compensation for TEMP < 20 °C (with a stronger correction
  // below -15 °C). Constants differ between MS5611 and MS5607.
  if (TEMP < 2000) {
    int32_t T2    = (int32_t)(((int64_t)dT * dT) >> 31);
    int64_t d     = (int64_t)(TEMP - 2000) * (TEMP - 2000);
    int64_t OFF2, SENS2;
#if MS56XX_IS_MS5607
    OFF2  = (61 * d) >> 4;
    SENS2 = 2 * d;
#else
    OFF2  = (5 * d) >> 1;
    SENS2 = (5 * d) >> 2;
#endif
    if (TEMP < -1500) {
      int64_t e = (int64_t)(TEMP + 1500) * (TEMP + 1500);
#if MS56XX_IS_MS5607
      OFF2  += 15 * e;
      SENS2 += 8 * e;
#else
      OFF2  += 7 * e;
      SENS2 += (11 * e) >> 1;
#endif
    }
    TEMP -= T2;
    OFF  -= OFF2;
    SENS -= SENS2;
  }

  int64_t P64 = ((((int64_t)D1 * SENS) >> 21) - OFF) >> 15;

  temp_c      = TEMP / 100.0f;       // 0.01 °C units -> °C
  pressure_pa = (float)P64;          // datasheet result is in 0.01 mbar = Pa
  altitude_m  = 44330.0f * (1.0f - powf(pressure_pa / SEA_LEVEL_PRESSURE_PA, 0.1903f));
}

void sendSensorData() {
  float t = 0, p = 0, a = 0;
  readMS5611(t, p, a);

  Serial.println("=== MS5611 Sensor ===");
  Serial.print("Temperature: "); Serial.print(t, 2); Serial.println(" C");
  Serial.print("Pressure:    "); Serial.print(p, 1); Serial.println(" Pa");
  Serial.print("Altitude:    "); Serial.print(a+18, 2); Serial.println(" m");

  Serial.println("Data sent successfully");
}

// ----- NMEA helpers -----

// Return the n-th comma-separated field (0-based). Stops at '*' or end.
static String nmeaField(const String &s, int idx) {
  int start = 0;
  for (int i = 0; i < idx; i++) {
    int next = s.indexOf(',', start);
    if (next < 0) return "";
    start = next + 1;
  }
  int end = s.indexOf(',', start);
  int star = s.indexOf('*', start);
  if (end < 0 || (star >= 0 && star < end)) end = star;
  if (end < 0) end = s.length();
  return s.substring(start, end);
}

// Parse "ddmm.mmmm..." (or "dddmm.mmmm..." for longitude) into 1E-7 degrees.
static int32_t nmeaToE7(const String &val, char hemi) {
  if (val.length() == 0) return 0;
  int dot = val.indexOf('.');
  String intPart = (dot < 0) ? val : val.substring(0, dot);
  long dm = intPart.toInt();          // ddmm
  long deg = dm / 100;
  long minInt = dm % 100;
  double frac = (dot < 0) ? 0.0 : val.substring(dot).toDouble();  // 0.mmmm
  double dec = (double)deg + ((double)minInt + frac) / 60.0;
  if (hemi == 'S' || hemi == 'W') dec = -dec;
  return (int32_t)lround(dec * 1e7);
}

// Compute UNIX epoch microseconds from RMC date "ddmmyy" + time "hhmmss.ss".
static uint64_t toUnixUs(const String &date, const String &time) {
  if (date.length() < 6 || time.length() < 6) return 0;
  int dd = (date[0]-'0')*10 + (date[1]-'0');
  int mn = (date[2]-'0')*10 + (date[3]-'0');
  int yy = (date[4]-'0')*10 + (date[5]-'0');
  int hh = (time[0]-'0')*10 + (time[1]-'0');
  int mm = (time[2]-'0')*10 + (time[3]-'0');
  int ss = (time[4]-'0')*10 + (time[5]-'0');
  uint32_t us = 0;
  if (time.length() > 6 && time[6] == '.') {
    us = (uint32_t)(time.substring(6).toFloat() * 1e6);
  }
  int year = 2000 + yy;
  long days = 0;
  for (int y = 1970; y < year; y++) {
    bool leap = (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
    days += leap ? 366 : 365;
  }
  static const int dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  for (int m = 1; m < mn; m++) {
    days += dim[m-1];
    if (m == 2 && (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0))) days++;
  }
  days += dd - 1;
  uint64_t secs = (uint64_t)days * 86400ULL + (uint64_t)hh * 3600 + (uint64_t)mm * 60 + ss;
  return secs * 1000000ULL + us;
}

// $GPRMC / $GNRMC — UTC time, status (A/V), lat, N/S, lon, E/W, speed(kn), cog, date
void parseRMC(const String &line) {
  String time   = nmeaField(line, 1);
  String status = nmeaField(line, 2);
  String latS   = nmeaField(line, 3);
  String latD   = nmeaField(line, 4);
  String lonS   = nmeaField(line, 5);
  String lonD   = nmeaField(line, 6);
  String spd    = nmeaField(line, 7);
  String cog    = nmeaField(line, 8);
  String date   = nmeaField(line, 9);

  if (status == "A") {
    g_gps.lat_e7   = nmeaToE7(latS, latD.length() ? latD[0] : 'N');
    g_gps.lon_e7   = nmeaToE7(lonS, lonD.length() ? lonD[0] : 'E');
    g_gps.vel_cms  = (uint16_t)(spd.toFloat() * 51.444f);   // knots -> cm/s
    g_gps.cog_cdeg = (uint16_t)(cog.toFloat() * 100.0f);
    if (g_gps.fix_type < 2) g_gps.fix_type = 3;             // assume 3D until GGA refines
  } else {
    g_gps.fix_type = 1;                                      // NO_FIX
    g_gps.vel_cms = 0;
    g_gps.cog_cdeg = 0;
  }

  uint64_t t = toUnixUs(date, time);
  if (t > 0) { g_gps.time_unix_us = t; g_gps.have_time = true; }
}

// $GPGGA / $GNGGA — UTC time, lat, N/S, lon, E/W, fixQual, sats, HDOP, alt(M)
void parseGGA(const String &line) {
  String latS = nmeaField(line, 2);
  String latD = nmeaField(line, 3);
  String lonS = nmeaField(line, 4);
  String lonD = nmeaField(line, 5);
  String qual = nmeaField(line, 6);
  String sats = nmeaField(line, 7);
  String hdop = nmeaField(line, 8);
  String alt  = nmeaField(line, 9);

  int q = qual.toInt();
  // NMEA fix quality -> MAVLink GPS_FIX_TYPE
  switch (q) {
    case 1:  g_gps.fix_type = 3; break;  // 3D
    case 2:  g_gps.fix_type = 4; break;  // DGPS
    case 4:  g_gps.fix_type = 6; break;  // RTK fixed
    case 5:  g_gps.fix_type = 5; break;  // RTK float
    default: g_gps.fix_type = (q == 0) ? 1 : 1; break; // NO_FIX
  }
  g_gps.sats = (uint8_t)sats.toInt();
  g_gps.eph  = (uint16_t)(hdop.toFloat() * 100.0f);
  if (q > 0) {
    g_gps.lat_e7 = nmeaToE7(latS, latD.length() ? latD[0] : 'N');
    g_gps.lon_e7 = nmeaToE7(lonS, lonD.length() ? lonD[0] : 'E');
    g_gps.alt_mm = (int32_t)(alt.toFloat() * 1000.0f);
  }
}

void handleNmeaLine(const String &line) {
  if (line.startsWith("$GPGGA") || line.startsWith("$GNGGA")) {
    parseGGA(line);
  } else if (line.startsWith("$GPRMC") || line.startsWith("$GNRMC")) {
    parseRMC(line);
    sendGpsRawInt();   // RMC arrives last in the per-second NMEA burst
  }
}

// ----- MAVLink v1 GPS_RAW_INT (msg id 24, payload 30 bytes, CRC_EXTRA 24) -----

static inline void mav_crc_accumulate(uint8_t data, uint16_t *crc) {
  uint8_t tmp = data ^ (uint8_t)(*crc & 0xff);
  tmp ^= (tmp << 4);
  *crc = (*crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4);
}

static inline void put_u16(uint8_t *p, uint16_t v) { p[0] = v & 0xff; p[1] = (v >> 8) & 0xff; }
static inline void put_u32(uint8_t *p, uint32_t v) {
  p[0] = v & 0xff; p[1] = (v >> 8) & 0xff; p[2] = (v >> 16) & 0xff; p[3] = (v >> 24) & 0xff;
}
static inline void put_u64(uint8_t *p, uint64_t v) {
  for (int i = 0; i < 8; i++) p[i] = (v >> (8 * i)) & 0xff;
}

void sendGpsRawInt() {
  const uint8_t MSG_ID      = 24;
  const uint8_t PAYLOAD_LEN = 30;
  const uint8_t CRC_EXTRA   = 24;

  uint8_t buf[8 + PAYLOAD_LEN];
  buf[0] = 0xFE;                 // MAVLink v1 magic
  buf[1] = PAYLOAD_LEN;
  buf[2] = g_mav_seq++;
  buf[3] = MAV_SYSID;
  buf[4] = MAV_COMPID;
  buf[5] = MSG_ID;

  // Payload — fields are ordered largest-first per MAVLink wire format.
  uint8_t *p = &buf[6];
  put_u64(p,      g_gps.have_time ? g_gps.time_unix_us : (uint64_t)micros());  p += 8;
  put_u32(p,      (uint32_t)g_gps.lat_e7); p += 4;
  put_u32(p,      (uint32_t)g_gps.lon_e7); p += 4;
  put_u32(p,      (uint32_t)g_gps.alt_mm); p += 4;
  put_u16(p,      g_gps.eph);              p += 2;
  put_u16(p,      0xFFFF);                 p += 2;   // epv unknown
  put_u16(p,      g_gps.vel_cms);          p += 2;
  put_u16(p,      g_gps.cog_cdeg);         p += 2;
  *p++ = g_gps.fix_type;
  *p++ = g_gps.sats;

  // CRC over LEN..end-of-payload, then CRC_EXTRA.
  uint16_t crc = 0xFFFF;
  for (int i = 1; i < 6 + PAYLOAD_LEN; i++) mav_crc_accumulate(buf[i], &crc);
  mav_crc_accumulate(CRC_EXTRA, &crc);
  buf[6 + PAYLOAD_LEN]     = crc & 0xff;
  buf[6 + PAYLOAD_LEN + 1] = (crc >> 8) & 0xff;

  LoRa.beginPacket();
  LoRa.write(buf, sizeof(buf));
  LoRa.endPacket();
}