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
// #define LED_FEQ1 5
// #define LED_FEQ2 6
#define LED_FEQ3 7
int NODE_ID =1;
const long frequency = 411E6;  // LoRa Frequency

const int csPin = 10;          // LoRa radio chip select
const int resetPin = 9;        // LoRa radio reset
const int irqPin = 2;          // change for your board; must be a hardware interrupt pin
SoftwareSerial mySerial(6, 5);
unsigned char pinStat[30];
// unsigned char handShake1[]={0x7c,0x53,0x0d,0x55,0xaa,0xaa,0xaa,0xaa,0x00,0x20,0xaa,0x01,0x0d,0x0d};//len=14
// unsigned char handShake2[]={0x7c,0x57,0x3a,0x4f,0x49,0x3f,0x0d};//len=7
unsigned char handShake1[]  ={0x7c,0x44,0x0d};
unsigned char handShake2[]  ={0x7c,0x53,0x0d};
unsigned char handShake3[]  ={0x7c,0x57,0x0d};
unsigned char handShakeSVG[]={0x4e,0x6f,0x64,0x33,0x2c,0x81,0x53,0x44,0x84};//len=9

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
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  digitalWrite(LED3,LOW);
  if(NODE_ID==1){digitalWrite(LED2,HIGH);digitalWrite(LED3,LOW);}
  if(NODE_ID==2){digitalWrite(LED3,HIGH);digitalWrite(LED2,LOW);}
  // if(NODE_ID==3)digitalWrite(LED3,HIGH);
 
}
uint8_t loraSendBuf[255];
uint8_t loraReportBuf[20];
int buffCurPos=1;
unsigned long long int lastTimeSend=0;
unsigned long long int lastTimeRead=0;
int bt1_state=1000;
int bt2_state=1000;
void SetupSVG2()
{
  mySerial.begin(9600); 
  NODE_ID = 2;
  initLed();
  setupLora();
}
void SetupRaidM100()
{
  mySerial.begin(57600); 
  NODE_ID = 1;
  initLed();
  setupLora();
}
void setupLora()
{
  loraSendBuf[0] = NODE_ID;
  for(int i=0;i<20;i++)
  {
    loraReportBuf[i]=0;
  }
  loraReportBuf[0] =0x00;//indicator of report message
  loraReportBuf[1] =NODE_ID;
  
  // pinMode(BT1,INPUT_PULLUP);
  // pinMode(BT2,INPUT_PULLUP);
  digitalWrite(RF_EN, HIGH);
  delay(1000);
  // mySerial.println("Sytem init succeeded.");
  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(frequency)) {
    mySerial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }


  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
  digitalWrite( LED_DATA ,LOW);
  digitalWrite( LED_MODE ,LOW);
  digitalWrite( LED_FEQ3 ,LOW);
}
void setup() {
  // mySerial.begin(57600);    
  SetupRaidM100();
  
}

void loop() {
  // send data if buffer overflow
  while(mySerial.available())
  {
    char input = mySerial.read();
    lastTimeRead=millis();
    // mySerial.write(input);
    loraSendBuf[buffCurPos]=input;
    buffCurPos++;
    if(buffCurPos>=254)
    {
      LoRa_sendData(loraSendBuf,buffCurPos);
      buffCurPos=1;
    }
  }
  // send data if timeout
  if((buffCurPos>1)&&((millis()-lastTimeRead)>200))
  {
    if(millis()-lastTimeSend>200)LoRa_sendData(loraSendBuf,buffCurPos);
      buffCurPos=1;
  }
  // check button press
  int newbt1 = analogRead(BT1);
  bt1_state+=((newbt1-bt1_state)*0.1);
  if(bt1_state<100)
  {
    SetupSVG2();
    delay(100);
  }
  int newbt2 = analogRead(BT2);
  bt2_state+=((newbt2-bt2_state)*0.1);
  if(bt2_state<100)
  {
    
    SetupRaidM100();
    delay(100);
  }
  if(NODE_ID==1)runRaidM100();
  else if(NODE_ID==2)runSVG();
}
void runSVG()
{
  if (runEvery(1000)) { // repeat every 1000 millis
    int time_sec = millis()/1000;
    int preiod_operation = 10;
    if((time_sec%preiod_operation)==0)
    {
      mySerial.write(handShakeSVG,9);
    }
    else if((time_sec%preiod_operation)==1)
    {
        loraReportBuf[2]=time_sec/256;
        loraReportBuf[3]= (unsigned char)time_sec;
        LoRa_sendData(loraReportBuf,20);
    }

  }
}
void runRaidM100()
{
  if (runEvery(1000)) { // repeat every 1000 millis
    int time_sec = millis()/1000;
    int preiod_operation = 4;
    if((time_sec%preiod_operation)==0)
    {
      mySerial.write(handShake1,3);
    }
    else if((time_sec%preiod_operation)==1)
    {
      mySerial.write(handShake2,3);
    }
    else if((time_sec%preiod_operation)==2)
    {
      mySerial.write(handShake3,3);
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


