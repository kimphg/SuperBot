#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

const int csPin = 10;
const int resetPin = 9;
const int irqPin = 8;
const int heatPin = 6; // Chân điều khiển trở sưởi D6

// --- CÁC HẰNG SỐ CÀI ĐẶT ĐIỀU KIỆN ---
const unsigned long TIME_LIMIT_MS = 5UL * 60UL * 60UL * 1000UL; // 5 tiếng = 18.000.000 ms
const double DISTANCE_LIMIT_M = 30000.0; // 30 km = 30.000 mét

SoftwareSerial gpsSerial(4, 3); // RX=D4, TX=D3
TinyGPSPlus gps;

Adafruit_BMP085 bmp;
unsigned int packetCounter = 0;

// --- CÁC BIẾN LƯU TRỮ TRẠNG THÁI ---
double startLat = 0.0;
double startLon = 0.0;
bool hasStartLocation = false; 
bool isHeating = false;        

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Sử dụng macro F() để tiết kiệm tối đa SRAM cho vi điều khiển
  Serial.println(F("========================================="));
  Serial.println(F("      KHOI DONG MACH PHAT (SENDER)       "));
  Serial.println(F("========================================="));

  pinMode(heatPin, OUTPUT);
  digitalWrite(heatPin, LOW); 
  Serial.println(F("[+] Chan D6 (Nung tro) da san sang. Trang thai: TAT"));

  gpsSerial.begin(9600);
  Serial.println(F("[+] GPS (u-blox M8N) tren D4(RX)/D3(TX)."));

  if (!bmp.begin()) {
    Serial.println(F("[-] LOI: Khong tim thay cam bien GY-68!"));
    while (1);
  }
  Serial.println(F("[+] Cam bien GY-68 (BMP180) san sang."));

  LoRa.setPins(csPin, resetPin, irqPin);
  if (!LoRa.begin(433E6)) {
    Serial.println(F("[-] LOI: Khong khoi dong duoc Module LoRa!"));
    while (1);
  }
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setCodingRate4(8);
  LoRa.setTxPower(20);
  Serial.println(F("[+] Module LoRa RA-02 san sang phat (433MHz)."));
  Serial.println(F("-----------------------------------------"));
}

void loop() {
  // 1. CHỜ VÀ ĐỌC GPS ĐẦU TIÊN (3 Giây)
  // Việc đọc GPS ngay đầu vòng lặp giúp dữ liệu không bị đứt gãy do quá trình phát LoRa
  unsigned long startDelay = millis();
  do {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }
  } while (millis() - startDelay < 5000);

  // 2. Đọc cảm biến BMP180
  float temperature = bmp.readTemperature();
  int32_t pressure  = bmp.readPressure();
  float bmpAltitude = bmp.readAltitude();

  // 3. Trích xuất dữ liệu GPS ngay khi vừa đọc xong
  String lat    = gps.location.isValid() ? String(gps.location.lat(), 6) : "N/A";
  String lon    = gps.location.isValid() ? String(gps.location.lng(), 6) : "N/A";
  String gpsAlt = gps.altitude.isValid() ? String(gps.altitude.meters(), 1) : "N/A";
  String speed  = gps.speed.isValid()  ? String(gps.speed.kmph(), 1) : "N/A";
  String course = gps.course.isValid() ? String(gps.course.deg(), 1) : "N/A";

  // =========================================================================
  // 4. LOGIC ĐIỀU KHIỂN NUNG TRỞ (CHÂN D6)
  // =========================================================================

  if (!hasStartLocation && gps.location.isValid()) {
    startLat = gps.location.lat();
    startLon = gps.location.lng();
    hasStartLocation = true;
    Serial.print(F("\n[!] DA LUU TOA DO XUAT PHAT: "));
    Serial.print(startLat, 6);
    Serial.print(F(", "));
    Serial.println(startLon, 6);
  }

  unsigned long currentUptime = millis();
  double currentDistance = 0.0;
  
  if (hasStartLocation && gps.location.isValid()) {
    currentDistance = TinyGPSPlus::distanceBetween(
      startLat, startLon,
      gps.location.lat(), gps.location.lng()
    );
  }

  if (!isHeating) {
    bool overTime = (currentUptime >= TIME_LIMIT_MS);
    bool overDistance = (hasStartLocation && currentDistance >= DISTANCE_LIMIT_M);

    if (overTime || overDistance) {
      digitalWrite(heatPin, HIGH); 
      isHeating = true;
      Serial.println(F("\n[!!!] KICH HOAT NUNG TRO CHIP!"));
    }
  }

  // =========================================================================

  packetCounter++;

  // 5. Đóng gói và phát dữ liệu
  String heatStatusStr = isHeating ? "ON" : "OFF";
  String payload = String(temperature, 1) + "|"
                 + String(pressure)       + "|"
                 + String(bmpAltitude, 1) + "|"
                 + lat    + "|"
                 + lon    + "|"
                 + gpsAlt + "|"
                 + speed  + "|"
                 + course + "|"
                 + heatStatusStr;

  Serial.print(F("["));
  Serial.print(packetCounter);
  Serial.print(F("] Phat: "));
  Serial.println(payload);

  if (hasStartLocation) {
    Serial.print(F(" -> Cach diem xuat phat: "));
    Serial.print(currentDistance, 1);
    Serial.println(F(" met"));
  }

  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();
  
  Serial.println(F("--> [OK] Da phat xong."));
  Serial.println(F("-----------------------------------------"));
}