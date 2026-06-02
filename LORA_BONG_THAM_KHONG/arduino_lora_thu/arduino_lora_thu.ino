#include <SPI.h>
#include <LoRa.h>
#include <math.h>

const int csPin = 10;
const int resetPin = 9;
const int irqPin = 8;

// --- NGUONG VA TRANG THAI BAY ---
const float ALT_THRESHOLD_M = 300.0;      // chi dem khi do cao > 300m
const unsigned long MAX_DELTA_MS = 30000; // bo qua khoang cach > 30s (mat goi)

double prevLat = 0.0;
double prevLon = 0.0;
bool hasPrevPosition = false;
bool wasAboveThreshold = false;
unsigned long lastPacketMs = 0;
unsigned long travelTimeMs = 0;
double totalDistanceM = 0.0;

double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6378137.0;
  double phi1 = radians(lat1);
  double phi2 = radians(lat2);
  double dphi = radians(lat2 - lat1);
  double dlmb = radians(lon2 - lon1);
  double a = sin(dphi / 2) * sin(dphi / 2)
           + cos(phi1) * cos(phi2) * sin(dlmb / 2) * sin(dlmb / 2);
  return 2.0 * R * asin(sqrt(a));
}

void formatDuration(unsigned long ms, char *out) {
  unsigned long s = ms / 1000UL;
  unsigned long h = s / 3600UL;
  unsigned long m = (s % 3600UL) / 60UL;
  unsigned long sec = s % 60UL;
  sprintf(out, "%02lu:%02lu:%02lu", h, m, sec);
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("--- KHOI DONG MACH NHAN LORA ---");

  LoRa.setPins(csPin, resetPin, irqPin);
  if (!LoRa.begin(433E6)) {
    Serial.println("Loi: Khong khoi dong duoc Module LoRa!");
    while (1);
  }
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setCodingRate4(8);

  Serial.println("-> Dang cho tin hieu tu mach phat...");
  Serial.println("-------------------------------------------------");
}

String getField(const String& data, int index) {
  int start = 0;
  for (int i = 0; i < index; i++) {
    start = data.indexOf('|', start) + 1;
    if (start == 0) return "N/A";
  }
  int end = data.indexOf('|', start);
  return (end == -1) ? data.substring(start) : data.substring(start, end);
}

String getDirection(float course) {
  if (course >= 337.5 || course < 22.5)  return "Bac (N)";
  if (course >= 22.5  && course < 67.5)  return "Dong Bac (NE)";
  if (course >= 67.5  && course < 112.5) return "Dong (E)";
  if (course >= 112.5 && course < 157.5) return "Dong Nam (SE)";
  if (course >= 157.5 && course < 202.5) return "Nam (S)";
  if (course >= 202.5 && course < 247.5) return "Tay Nam (SW)";
  if (course >= 247.5 && course < 292.5) return "Tay (W)";
  if (course >= 292.5 && course < 337.5) return "Tay Bac (NW)";
  return "N/A";
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String receivedData = "";
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }

    Serial.println("=== GOI TIN MOI ===");
    Serial.println("Raw: [" + receivedData + "]");

    // Lấy 9 trường dữ liệu (Thêm trường HeatStatus ở vị trí số 8)
    String temp     = getField(receivedData, 0);
    String pressure = getField(receivedData, 1);
    String bmpAlt   = getField(receivedData, 2);
    String lat      = getField(receivedData, 3);
    String lon      = getField(receivedData, 4);
    String gpsAlt   = getField(receivedData, 5);
    String speed    = getField(receivedData, 6);
    String course   = getField(receivedData, 7);
    String heatStat = getField(receivedData, 8); // Lấy trạng thái sưởi

    String directionStr = "N/A";
    if (course != "N/A" && course != "") {
      directionStr = getDirection(course.toFloat()) + " (" + course + " do)";
    }

    // --- CAP NHAT THONG KE BAY ---
    // Uu tien GPS altitude, neu khong thi dung BMP altitude
    float altMeters = NAN;
    if (gpsAlt != "N/A" && gpsAlt.length() > 0)      altMeters = gpsAlt.toFloat();
    else if (bmpAlt != "N/A" && bmpAlt.length() > 0) altMeters = bmpAlt.toFloat();

    bool isAbove = !isnan(altMeters) && altMeters > ALT_THRESHOLD_M;
    unsigned long nowMs = millis();

    if (isAbove && wasAboveThreshold && lastPacketMs != 0) {
      unsigned long delta = nowMs - lastPacketMs;
      if (delta > 0 && delta < MAX_DELTA_MS) {
        travelTimeMs += delta;
      }
    }
    wasAboveThreshold = isAbove;
    lastPacketMs = nowMs;

    if (isAbove && lat != "N/A" && lon != "N/A"
        && lat.length() > 0 && lon.length() > 0) {
      double curLat = lat.toDouble();
      double curLon = lon.toDouble();
      if (hasPrevPosition) {
        totalDistanceM += haversineMeters(prevLat, prevLon, curLat, curLon);
      }
      prevLat = curLat;
      prevLon = curLon;
      hasPrevPosition = true;
    } else if (!isAbove) {
      // Duoi nguong -> reset diem mocs de chi cong khoang cach trong khi bay
      hasPrevPosition = false;
    }

    char travelBuf[12];
    formatDuration(travelTimeMs, travelBuf);

    Serial.println("  Nhiet do      : " + temp + " C");
    Serial.println("  Ap suat       : " + pressure + " Pa");
    Serial.println("  Do cao BMP    : " + bmpAlt + " m");
    Serial.println("  Lat (GPS)     : " + lat);
    Serial.println("  Lon (GPS)     : " + lon);
    Serial.println("  Alt (GPS)     : " + gpsAlt + " m");
    Serial.println("  Van toc       : " + speed + " km/h");
    Serial.println("  Huong di      : " + directionStr);
    Serial.println("  SUOI (D6)     : " + heatStat);
    Serial.println("  RSSI          : " + String(LoRa.packetRssi()) + " dBm");
    Serial.print(F("  Thoi gian bay : "));
    Serial.print(travelBuf);
    Serial.println(F(" (chi dem khi >300m)"));
    Serial.print(F("  Quang duong   : "));
    if (totalDistanceM < 1000.0) {
      Serial.print(totalDistanceM, 1);
      Serial.println(F(" m"));
    } else {
      Serial.print(totalDistanceM / 1000.0, 2);
      Serial.println(F(" km"));
    }
    Serial.println("--------------------------------------------------");
  }
}