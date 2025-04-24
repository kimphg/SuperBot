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

#include <SPI.h>              // include libraries
#include <LoRa.h>

const long frequency = 411E6;  // LoRa Frequency

const int csPin = 10;          // LoRa radio chip select
const int resetPin = 9;        // LoRa radio reset
const int irqPin = 2;          // change for your board; must be a hardware interrupt pin
#define LED_DATA 8
#define LED_MODE 3
#define LED_FEQ1 5
#define LED_FEQ2 6
#define LED_FEQ3 7
void setup() {
  pinMode( LED_DATA ,OUTPUT);
  pinMode( LED_MODE ,OUTPUT);
  pinMode( LED_FEQ1 ,OUTPUT);
  pinMode( LED_FEQ2 ,OUTPUT);
  pinMode( LED_FEQ3 ,OUTPUT);
  digitalWrite( LED_DATA ,HIGH);
  digitalWrite( LED_MODE ,HIGH);
  digitalWrite( LED_FEQ1 ,HIGH);
  digitalWrite( LED_FEQ2 ,HIGH);
  digitalWrite( LED_FEQ3 ,HIGH);
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");
  Serial.println();
  Serial.println("LoRa Simple Node");
  Serial.println("Only receive messages from gateways");
  Serial.println("Tx: invertIQ disable");
  Serial.println("Rx: invertIQ enable");
  Serial.println();

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
  digitalWrite( LED_DATA ,LOW);
  digitalWrite( LED_MODE ,LOW);
  digitalWrite( LED_FEQ1 ,HIGH);
  digitalWrite( LED_FEQ2 ,HIGH);
  digitalWrite( LED_FEQ3 ,LOW);
}
uint8_t loraSendBuf[200];
int buffCurPos=0;
unsigned long long int lastTimeSend=0;
void loop() {
  while(Serial.available())
  {
    loraSendBuf[buffCurPos]=Serial.read();
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
  if (runEvery(5000)) { // repeat every 1000 millis

    String message = "Node1:";
    message += millis()/1000;

    LoRa_sendMessage(message); // send a message
// Serial.print(message);
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
  digitalWrite( LED_MODE ,HIGH);
}
void LoRa_sendData(uint8_t* data,int len) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.write(data,len);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
  lastTimeSend=millis();
  digitalWrite( LED_MODE ,HIGH);
}
void onReceive(int packetSize) {
  digitalWrite(LED_DATA,HIGH);

  while (LoRa.available()) {
    Serial.write(LoRa.read());
  }
  digitalWrite(LED_DATA,LOW);


}

void onTxDone() {
  // Serial.println("TxDone");
  
  LoRa_rxMode();
  digitalWrite( LED_MODE ,LOW);
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

