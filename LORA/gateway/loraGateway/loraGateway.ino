/*
  LoRa Simple Gateway/Node Exemple

  This code uses InvertIQ function to create a simple Gateway/Node logic.

  Gateway - Sends messages with enableInvertIQ()
          - Receives messages with disableInvertIQ()

  Node    - Sends messages with disableInvertIQ()
          - Receives messages with enableInvertIQ()

  With this arrangement a Gateway never receive messages from another Gateway
  and a Node never receive message from another Node.
  Only Gateway to Node and vice versa.

  This code receives messages and sends a message every second.

  InvertIQ function basically invert the LoRa I and Q signals.

  See the Semtech datasheet, http://www.semtech.com/images/datasheet/sx1276.pdf
  for more on InvertIQ register 0x33.

  created 05 August 2018
  by Luiz H. Cassettari
*/
#include <SoftwareSerial.h>
#include <SPI.h>              // include libraries
#include <LoRa.h>

const long frequency = 411E6;  // LoRa Frequency

SoftwareSerial mySerial(6, 5);
const int csPin = 10;          // LoRa radio chip select
const int resetPin = 9;        // LoRa radio reset
const int irqPin = 2;          // change for your board; must be a hardware interrupt pin

void setup() {
  mySerial.begin(115200);                   // initialize serial
  while (!Serial);

  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  mySerial.println("LoRa init succeeded.");
  mySerial.println();
  mySerial.println("LoRa Simple Gateway");
  mySerial.println("Only receive messages from nodes");
  mySerial.println("Tx: invertIQ enable");
  mySerial.println("Rx: invertIQ disable");
  mySerial.println();

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
}
uint8_t loraSendBuf[200];
int buffCurPos=0;
unsigned long long int lastTimeSend=0;
void loop() {
  while(mySerial.available())
  {
    loraSendBuf[buffCurPos]=mySerial.read();
    buffCurPos++;
    if(buffCurPos>=199)
    {
      LoRa_sendData(loraSendBuf,buffCurPos);
      buffCurPos=0;
    }
  }
  if((buffCurPos>0)&&(millis()-lastTimeSend>100))
  {
    LoRa_sendData(loraSendBuf,buffCurPos);
      buffCurPos=0;
  }
  // if (runEvery(5000)) { 
  //   // repeat every 5000 millis
  //   Serial.println("Send Message!");
  // }
}

void LoRa_rxMode(){
  LoRa.disableInvertIQ();               // normal mode
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.enableInvertIQ();                // active invert I and Q signals
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}
void LoRa_sendData(uint8_t* data,int len) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.write(data,len);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
  lastTimeSend=millis();
}
void onReceive(int packetSize) {
  // String message = "";
  mySerial.write("\r\nmsg:");
  while (LoRa.available()) {
    mySerial.write(LoRa.read());
  }

  // Serial.print("Gateway Receive: ");
  // Serial.println(message);
  // Serial.println("RSSI: " + String(LoRa.packetRssi()));
  // Serial.println("Snr: " + String(LoRa.packetSnr()));
  // Serial.println();
}

void onTxDone() {
  // Serial.println("TxDone");
  LoRa_rxMode();
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

