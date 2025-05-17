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

minicore
328pb
1mhz external 
*/
#include <SoftwareSerial.h>
#include <SPI.h>              // include libraries
#include <LoRa.h>
#define  RF_EN 3
#define  LED3 A3
#define  LED2 A2
#define  LED1 4
#define  BT1 A6
#define  BT2 A7
#define LED_DATA 8
#define LED_MODE LED1
#define LED_FEQ1 5
#define LED_FEQ2 6
#define LED_FEQ3 7
#define NODE_ID 1
const long frequency = 411E6;  // LoRa Frequency

const int csPin = 10;          // LoRa radio chip select
const int resetPin = 9;        // LoRa radio reset
const int irqPin = 2;          // change for your board; must be a hardware interrupt pin
SoftwareSerial mySerial(6, 5);
unsigned char pinStat[30];
unsigned char handShake1[]={0x7c,0x53,0x0d};
unsigned char handShake2[]={0x7c,0x44,0x0d};
int pinDelay[30];

bool checkInput(int pinID)
{
  if(pinDelay[pinID]>100)
  {
    pinDelay[pinID]--;
    return false;
  }
  int newstat =digitalRead(pinID);
  if(newstat!=pinStat[pinID])
  {
    pinDelay[pinID]=1000;
    pinStat[pinID]=newstat;
    return true;
  }
  return false;

}
void initLed()
{
    pinMode(RF_EN,OUTPUT);
  
  pinMode(LED3,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(LED1,OUTPUT);
  pinMode(LED_DATA,OUTPUT);
  pinMode(LED_MODE,OUTPUT);
  pinMode(LED_FEQ1,OUTPUT);
  pinMode(LED_FEQ2,OUTPUT);
  pinMode(LED_FEQ3,OUTPUT);
  delay(10);
  digitalWrite(RF_EN,HIGH);
  digitalWrite(LED3,HIGH);
  digitalWrite(LED2,HIGH);
  digitalWrite(LED1,HIGH);
  digitalWrite(LED_DATA,HIGH);
  digitalWrite(LED_MODE,HIGH);
  digitalWrite(LED_FEQ1,HIGH);
  digitalWrite(LED_FEQ2,HIGH);
  digitalWrite(LED_FEQ3,HIGH);
  delay(500);
  digitalWrite(RF_EN,LOW);
  digitalWrite(LED3,LOW);
  digitalWrite(LED2,LOW);
  digitalWrite(LED1,LOW);
  digitalWrite(LED_DATA,LOW);
  digitalWrite(LED_MODE,LOW);
  digitalWrite(LED_FEQ1,LOW);
  digitalWrite(LED_FEQ2,LOW);
  digitalWrite(LED_FEQ3,LOW);
  delay(500);
  digitalWrite(RF_EN,HIGH);
  digitalWrite(LED3,HIGH);
  digitalWrite(LED2,HIGH);
  digitalWrite(LED1,HIGH);
  digitalWrite(LED_DATA,HIGH);
  digitalWrite(LED_MODE,HIGH);
  digitalWrite(LED_FEQ1,HIGH);
  digitalWrite(LED_FEQ2,HIGH);
  digitalWrite(LED_FEQ3,HIGH);
  delay(500);
  digitalWrite(RF_EN,LOW);
  digitalWrite(LED3,LOW);
  digitalWrite(LED2,LOW);
  digitalWrite(LED1,LOW);
  digitalWrite(LED_DATA,LOW);
  digitalWrite(LED_MODE,LOW);
  digitalWrite(LED_FEQ1,LOW);
  digitalWrite(LED_FEQ2,LOW);
  digitalWrite(LED_FEQ3,LOW);


}
uint8_t loraSendBuf[200];
uint8_t loraReportBuf[20];
int buffCurPos=1;
unsigned long long int lastTimeSend=0;
void setup() {
  mySerial.begin(38400);    
  initLed();
  initLed();
  loraSendBuf[0] = NODE_ID;
  for(int i=0;i<20;i++)
  {
    loraReportBuf[i]=0;
  }
  loraReportBuf[0] =0x00;//indicator of report message
  loraReportBuf[1] =NODE_ID;
  
  pinMode(BT1,INPUT);
  pinMode(BT2,INPUT);
  digitalWrite(RF_EN, HIGH);
  delay(1000);
  mySerial.println("Sytem init succeeded.");
  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(frequency)) {
    mySerial.println("LoRa init failed. Check your connections.");
    // while (true);                       // if failed, do nothing
  }

  mySerial.println("LoRa init succeeded.");
  mySerial.println();
  mySerial.println("LoRa Simple Node");
  mySerial.println("Only receive messages from gateways");
  mySerial.println("Tx: invertIQ disable");
  mySerial.println("Rx: invertIQ enable");
  mySerial.println();

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
  digitalWrite( LED_DATA ,LOW);
  digitalWrite( LED_MODE ,LOW);
  digitalWrite( LED_FEQ1 ,HIGH);
  digitalWrite( LED_FEQ2 ,HIGH);
  digitalWrite( LED_FEQ3 ,LOW);
}

void loop() {
  // send data if buffer overflow
  while(mySerial.available())
  {
    char input = mySerial.read();
    // mySerial.write(input);
    loraSendBuf[buffCurPos]=input;
    buffCurPos++;
    if(buffCurPos>=199)
    {
      LoRa_sendData(loraSendBuf,buffCurPos);
      buffCurPos=1;
    }
  }
  // send data if timeout
  if((buffCurPos>1)&&(millis()-lastTimeSend>10))
  {
    LoRa_sendData(loraSendBuf,buffCurPos);
      buffCurPos=1;
  }
  // check button press
  if(checkInput(BT1))
  {
    if(pinStat[BT1]==HIGH)
    {
      pinStat[LED1] = !pinStat[LED1];
      digitalWrite(LED1, pinStat[LED1]);
      mySerial.println(pinStat[LED1]);
    }
  }
  if (runEvery(1000)) { // repeat every 1000 millis
    int time_sec = millis()/1000;

    if(time_sec%3==0)
    {
      mySerial.write(handShake1,3);
    }
    else if(time_sec%3==1)
    {
      mySerial.write(handShake2,3);
    }
    else {
        loraReportBuf[2]=time_sec/256;
        loraReportBuf[3]= (unsigned char)time_sec;
        LoRa_sendData(loraReportBuf,20);
    }
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


