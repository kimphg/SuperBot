#include <SoftwareSerial.h>
#include <SPI.h>              // include libraries
#include <LoRa.h>

int NODE_ID = 1;
const long frequency = 411E6;  // LoRa Frequency

const int csPin = 10;          // LoRa radio chip select
const int resetPin = 9;        // LoRa radio reset
const int irqPin = 2;          // change for your board; must be a hardware interrupt pin
SoftwareSerial mySerial(6, 5);

// Bổ sung khai báo chân (Bạn có thể đổi số tùy mạch thực tế)
const int RF_EN = 7;
const int LED_DATA = 8;
const int LED_MODE = 3;
const int LED_FEQ3 = 4;

uint8_t loraSendBuf[255];
uint8_t loraReportBuf[20];
int buffCurPos = 1;

// Đổi từ 'long long int' sang 'long' cho đúng chuẩn millis()
unsigned long lastTimeSend = 0;
unsigned long lastTimeRead = 0;

volatile bool packetReceived = false; // Thêm cờ cho hàm ngắt

void setupLora()
{
  loraSendBuf[0] = NODE_ID;
  for(int i = 0; i < 20; i++)
  {
    loraReportBuf[i] = 0;
  }
  loraReportBuf[0] = 0x00; // indicator of report message
  loraReportBuf[1] = NODE_ID;
  
  // Bổ sung pinMode để digitalWrite hoạt động
  pinMode(RF_EN, OUTPUT);
  pinMode(LED_DATA, OUTPUT);
  pinMode(LED_MODE, OUTPUT);
  pinMode(LED_FEQ3, OUTPUT);

  digitalWrite(RF_EN, HIGH);
  delay(1000);
  
  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
  digitalWrite(LED_DATA, LOW);
  digitalWrite(LED_MODE, LOW);
  digitalWrite(LED_FEQ3, LOW);
}

void setup() {
  Serial.begin(115200);     // Khởi tạo Serial In ra máy tính
  mySerial.begin(57600);    // Bỏ comment để dùng mySerial
  setupLora();
}

void loop() {
  // Xử lý đọc LoRa an toàn ngoài hàm ngắt
  if (packetReceived) {
    digitalWrite(LED_DATA, HIGH);
    while (LoRa.available()) {
      Serial.write(LoRa.read());
    }
    digitalWrite(LED_DATA, LOW);
    packetReceived = false;
    Serial.println("da nhan");
  }

  // send data if buffer overflow
  while(mySerial.available())
  {
    char input = mySerial.read();
    lastTimeRead = millis();
    loraSendBuf[buffCurPos] = input;
    buffCurPos++;
    if(buffCurPos >= 254)
    {
      LoRa_sendData(loraSendBuf, buffCurPos);
      buffCurPos = 1;
    }
  }
  
  // send data if timeout
  if((buffCurPos > 1) && ((millis() - lastTimeRead) > 200))
  {
    if(millis() - lastTimeSend > 200) {
      LoRa_sendData(loraSendBuf, buffCurPos);
      buffCurPos = 1; // Sửa lỗi logic: Đưa vào trong khối if
    }
  }
  
  if (runEvery(1000)) { // repeat every 1000 millis
    int time_sec = millis() / 1000;
    loraReportBuf[2] = time_sec / 256;
    loraReportBuf[3] = (unsigned char)time_sec;
    LoRa_sendData(loraReportBuf, 20);
    Serial.println("da gui");
  }
}

void LoRa_rxMode(){
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
  digitalWrite(LED_MODE, HIGH);
}

void LoRa_sendData(uint8_t* data, int len) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.write(data, len);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
  lastTimeSend = millis();
  digitalWrite(LED_MODE, HIGH);
}

void onReceive(int packetSize) {
  packetReceived = true; // Sửa lỗi treo: Chỉ bật cờ báo hiệu có dữ liệu
}

void onTxDone() {
  LoRa_rxMode();
  digitalWrite(LED_MODE, LOW);
}

boolean runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}
