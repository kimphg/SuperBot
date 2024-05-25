//micro controller type: LGT8F328P
#define LIFT_MOTOR
// #define WHEEL_MOTOR_LEFT
// #define WHEEL_MOTOR_RIGHT
#ifdef WHEEL_MOTOR_RIGHT
#define MEN 2
#define REV 2
#define FWD 3
#define ALM_RES 4
#define SPEED 5
#define MB_FREE 6
#define MIN_LIM 7 //speed out
#define WNG_IN 8
#define ALARM_IN 9
#define MAX_LIM 10//emergency stop
#define DEBUG_TELEMETRY Serial
#define COMMAND_LEN_MAX 100
#define MOTOR_ID 1
#endif
#ifdef WHEEL_MOTOR_LEFT
#define MEN 2
#define REV 2
#define FWD 3
#define ALM_RES 4
#define SPEED 5
#define MB_FREE 6
#define MIN_LIM 7
#define WNG_IN 8
#define ALARM_IN 9
#define MAX_LIM 10
#define DEBUG_TELEMETRY Serial
#define COMMAND_LEN_MAX 100
#define MOTOR_ID 2
#endif
#ifdef LIFT_MOTOR
#define MEN 2
#define REV 2
#define FWD 3
#define ALM_RES 4
#define SPEED 5
#define MB_FREE 6
#define MIN_LIM 7
#define WNG_IN 8
#define ALARM_IN 9
#define MAX_LIM 10
#define MOTOR_ID 3
#endif
// #define DEBUG
#define LED_1 11
#define DEBUG_TELEMETRY Serial
#define COMMAND_LEN_MAX 50
#include <Arduino.h>
int output_speed=0;
int stackLevel = 0;
unsigned long int lastUpdate = 0;
unsigned int csFailCount=0;
unsigned int hdFailCount=0;
bool minOK=false, maxOK=false;
void setSpeed(int speed);
uint8_t commandBuff[COMMAND_LEN_MAX];
uint8_t reportPacket[8];
int commandBuffIndex = 0;
uint8_t last_byte = 0;
uint8_t calcCS8(uint8_t* startbyte, uint8_t len)
{
  int cs = 0;
  for (int i = 0; i < len; i++) {
    cs ^= startbyte[i];
  }
  return cs;
}
void blink(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
  }
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
}
bool updateBinaryCommand() {

  bool packetExecuted = false;
  while (DEBUG_TELEMETRY.available()) {
    uint8_t bytein = DEBUG_TELEMETRY.read();
    // Serial.println(bytein);
    if (commandBuffIndex < COMMAND_LEN_MAX-1) {
      commandBuff[commandBuffIndex] = bytein;
      commandBuffIndex++;
    } else 
    {
      commandBuffIndex = 0;
      commandBuff[commandBuffIndex] = bytein;
      commandBuffIndex++;
    }

    if (bytein == 0x55)  //end of command header
    {
      if (last_byte == 0xaa)  //check header
      {
        commandBuff[1] = 0x55;
        commandBuff[0] = 0xaa;
        commandBuffIndex = 2;
      }
    }
    last_byte = bytein;
    if (commandBuffIndex > 6) {
      if ((commandBuff[1] == 0x55)&&(commandBuff[0] == 0xaa) ){
          // int packetLen = 6;
          int motorID = commandBuff[2];
          if (MOTOR_ID == motorID) {
            
            uint8_t cs = calcCS8(commandBuff,6);
            
            if (cs == commandBuff[6]) {
#ifdef DEBUG
              Serial.print("Speed: ");
              Serial.println(speed);
#endif
            int motorDir = 0;
            if (commandBuff[4] == 0xAB) motorDir = 1;
            else if (commandBuff[4] == 0xBA) motorDir = -1;
            int speed = motorDir * commandBuff[3];
            int motorMode = commandBuff[5];
              setSpeed(speed);
              packetExecuted = true;
              csFailCount=0;
              // digitalWrite(13,LOW);
            }
            else 
            {
              csFailCount++;
              
            }
#ifdef DEBUG
            for (int i = 0; i < 6; i++) {
              Serial.print(commandBuff[i]);
              Serial.print(" ");
            }
            Serial.print("CS: ");
            Serial.println(cs);
            Serial.print("real CS: ");
            Serial.println(commandBuff[6]);
#endif
          }
          commandBuffIndex = 0;
          hdFailCount=0;
        } else {
          commandBuffIndex = 0;
          hdFailCount++;
#ifdef DEBUG
          Serial.print("wrong header: ");
          Serial.println(commandBuff[0]);
          Serial.println(commandBuff[1]);
#endif
        }
    }
    
  }
  digitalWrite(LED_1, packetExecuted);
  if(csFailCount>5){blink(5);
              csFailCount=0;}
  if(hdFailCount>10){blink(6);
              hdFailCount=0;}
  return packetExecuted;
}

void sendReport()
{
  delay(2+MOTOR_ID*2);
  reportPacket[2] = (0x80+MOTOR_ID);
  reportPacket[3] = ((abs(output_speed))&0xff);
  if(output_speed>=0)
  reportPacket[4] = (0xab);
  else reportPacket[4] = (0xba);
  reportPacket[5] = minOK;
  reportPacket[6] = maxOK;
  uint8_t cs=0;
  for (int i = 0; i < 7; i++) {
              cs ^= reportPacket[i];
            }
  reportPacket[7]=cs;
  Serial.write(reportPacket,8);
}
void setSpeed(int speed) {
  
  // if (speed != 0) digitalWrite(LED_BUILTIN, LOW);
  // else digitalWrite(LED_BUILTIN, HIGH);
  switch (MOTOR_ID) {
    case 1:  //WHEEL_MOTOR_RIGHT
      output_speed = speed;
      if (output_speed > 0) {
        digitalWrite(FWD, LOW);
        digitalWrite(REV, HIGH);
        analogWrite(SPEED, 255 - output_speed);
      } else {
        digitalWrite(FWD, HIGH);
        digitalWrite(REV, LOW);
        analogWrite(SPEED, 255 + output_speed);
      }
      break;
    case 2:  //WHEEL_MOTOR_LEFT
      output_speed = -speed;
      if (output_speed > 0) {
        digitalWrite(FWD, LOW);
        digitalWrite(REV, HIGH);
        analogWrite(SPEED, 255 - output_speed);
      } else {
        digitalWrite(FWD, HIGH);
        digitalWrite(REV, LOW);
        analogWrite(SPEED, 255 + output_speed);
      }
      break;
    case 3:  //LIFT_MOTOR
      output_speed = speed;
      
      
      if (!minOK)
        if (output_speed < 0) output_speed = 0;
      if (!maxOK)
        if (output_speed > 0) output_speed = 0;
      if(output_speed==0)digitalWrite(MEN, LOW);
      else digitalWrite(MEN, HIGH);
      if (output_speed > 0) {
        digitalWrite(FWD, LOW);
        analogWrite(SPEED, 255 - output_speed);
      } else {
        digitalWrite(FWD, HIGH);
        analogWrite(SPEED, 255 + output_speed);
      }
      break;
    default:
      break;
  }
}
void setup() {
  pinMode(MIN_LIM, INPUT);
  pinMode(WNG_IN, INPUT);
  pinMode(ALARM_IN, INPUT);
  pinMode(MAX_LIM, INPUT);
  pinMode(LED_1, OUTPUT);
  digitalWrite(LED_1, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(MEN, OUTPUT);
  digitalWrite(MEN, LOW);
  pinMode(FWD, OUTPUT);
  digitalWrite(FWD, LOW);
  pinMode(ALM_RES, OUTPUT);
  digitalWrite(ALM_RES, LOW);
  pinMode(SPEED, OUTPUT);
  digitalWrite(SPEED, HIGH);
  pinMode(MB_FREE, OUTPUT);
  digitalWrite(MB_FREE, HIGH);
  
  reportPacket[0]=0xAA;
  reportPacket[1]=0x55;
  blink(MOTOR_ID);
  Serial.begin(230400);

  setSpeed(0);
  digitalWrite(LED_1, LOW);
}

void loop() {
  maxOK = (digitalRead(MAX_LIM));
  minOK = (digitalRead(MIN_LIM));
  if((minOK&&maxOK)==false)digitalWrite(LED_BUILTIN,HIGH);
  else digitalWrite(LED_BUILTIN,LOW);
  if(!(minOK|maxOK))blink(4);
  if (updateBinaryCommand()) {
    lastUpdate = millis();
    sendReport();
  }
  
  if (millis() - lastUpdate > 1000)
  { setSpeed(0);//failsafe if no connection
    digitalWrite(LED_1, LOW);
    delay(200);sendReport();
  }
}
