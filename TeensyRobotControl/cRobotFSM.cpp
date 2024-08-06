#include "core_pins.h"
// #include "core_pins.h"
// #include "usb_serial.h"
// #include "wiring.h"
// #include "core_pins.h"
#include <stdint.h>

// #include "usb_serial.h"
#include <math.h>

#include "cRobotFSM.h"
float curSpeedLift2=0;
#define ROT_SPEED_BUFF_SIZE 30
float desRotSpdBuf[ROT_SPEED_BUFF_SIZE];
int desRotSpdBufi=0;
float getdesRotSpdDelay(float input,int delay)
{
  int idOut = desRotSpdBufi-delay;
  if(idOut<0)idOut+=ROT_SPEED_BUFF_SIZE;
  float output = desRotSpdBuf[idOut];
  desRotSpdBuf[desRotSpdBufi] = input;
  desRotSpdBufi++;
  if(desRotSpdBufi>=ROT_SPEED_BUFF_SIZE)desRotSpdBufi=0;
  return output;
}
void resetDesRotSpd()
{
  for (int i=0;i<ROT_SPEED_BUFF_SIZE;i++)
  {
    desRotSpdBuf[desRotSpdBufi]=0;
  }
  
}

long int encPosRight = 0;
long int encPosLeft = 0;
long int encRighto = 0;
long int encLefto = 0;
long int liftLevel = 0;
long int liftLevelo = 0;
// long int reportCount = 0;
float liftAngleErroro = 0;
float lift_gear_lash = 3.0;
bool liftLevelInitOK = false;
int desLiftLevel = 0;
long int M1ComTime = 0, M2ComTime = 0, M3ComTime = 0;
long int M1DelayTime,M2DelayTime,M3DelayTime;
std::vector<RobotParam> paramTable;
std::vector<FloorTag> floorMap;
#define INTEGRAL_LEN 100
float array_yaw[INTEGRAL_LEN];
float array_pos[INTEGRAL_LEN];
float array_lift[INTEGRAL_LEN];
int integral_id_pos=0;
int integral_id_yaw=0;
int integral_id_lift=0;
void resetIntegral()
{
  for(int i=0;i<INTEGRAL_LEN;i++)
  {
    array_pos[i]=0;
    array_yaw[i]=0;
    array_lift[i]=0;
  }
}
float getIntegralYaw()
{
  float res=0;
  for(int i=0;i<INTEGRAL_LEN;i++)
  {
    res+=array_yaw[i];
  }
  return res/INTEGRAL_LEN;
}
float getIntegralPos()
{
  float res=0;
  for(int i=0;i<INTEGRAL_LEN;i++)
  {
    res+=array_pos[i];
  }
  return res/INTEGRAL_LEN;
}
float getIntegralLift()
{
  float res=0;
  for(int i=0;i<INTEGRAL_LEN;i++)
  {
    res+=array_lift[i];
  }
  return res/INTEGRAL_LEN;
}
void setIntegralPos(float value)
{
    array_pos[integral_id_pos]=value;
    integral_id_pos++;
    if(integral_id_pos>=INTEGRAL_LEN)integral_id_pos=0;
}
void setIntegralLift(float value)
{
    array_lift[integral_id_lift]=value;
    integral_id_lift++;
    if(integral_id_lift>=INTEGRAL_LEN)integral_id_lift=0;
}
void setIntegralYaw(float value)
{
  array_yaw[integral_id_yaw]=value;
  integral_id_yaw++;
  if(integral_id_yaw>=INTEGRAL_LEN)integral_id_yaw=0;
}

#define CONTROL_LEN 10
// #define MAX_LIFT_H 12000
int liftMaxLevel = 0;
unsigned char controlPacket[CONTROL_LEN];
void encRightInt() {
  if (digitalRead(ENC_B1) > 0)
    encPosRight++;
  else encPosRight--;
}
void EncLeftInt() {
  if (digitalRead(ENC_B2) > 0)
    encPosLeft--;
  else encPosLeft++;
}
long int EncIntLiftTime = 0;
void EncIntLift() {
  // long int NewEncIntLiftTime  = micros();
  // long int dt = NewEncIntLiftTime-EncIntLiftTime;
  // if(dt<0)dt+=4294967295;
  // EncIntLiftTime=NewEncIntLiftTime;
  if (digitalRead(ENC_B3) > 0) {
    liftLevel++;
    // if(dt)curSpeedLift2 = 1000.0/LIFT_PPR/(dt/1000.0)*57.3;
  } else {
    liftLevel--;
    // if(dt)curSpeedLift2 = -1000.0/LIFT_PPR/(dt/1000.0)*57.3;
  }
  // if (liftLevel < 0) liftLevel = 0;
  // if(liftLevel>MAX_LIFT_H)liftLevel=MAX_LIFT_H;
}
float constrainVal(float input, float min, float max) {
  if (input < min) return min;
  if (input > max) return max;
  return input;
}
void addFloorTag(int id,int x,int y)
{
  for (unsigned int i=0;i<floorMap.size();i++)
  {
    if(floorMap[i].id==id)
    {
      floorMap[i].x = x;
      floorMap[i].y = y;
      return;
    }
    
  }
  FloorTag newfloorTag;
  newfloorTag.id=id;
  newfloorTag.x = x;
  newfloorTag.y = y;
  floorMap.push_back(newfloorTag);
}
FloorTag getFloorTag(int id)
{
  for (unsigned int i=0;i<floorMap.size();i++)
  {
    if(floorMap[i].id==id)
    {
      
      return floorMap[i];
    }
    
  }
  FloorTag newfloorTag;
  newfloorTag.id=-1;
  newfloorTag.x = 0;
  newfloorTag.y = 0;
  return newfloorTag;
}
bool stepStat =false;


unsigned long long timeStep = 0;
int clock_counter_us=0;
int pulse_period_us = 10000000;
bool pulse_state=false;
void RobotDriver::stepOutput(int dir )
{
  clock_counter_us++;
  if(clock_counter_us>pulse_period_us)
  {
    clock_counter_us=0;
    
    if(desLiftSpeed>0)
    {
      if(isLiftMaxPos )return;
      digitalWriteFast(STEP_DIR, HIGH);
    
      curLiftStep++;
    }
    else
    {
      if(isLiftMinPos)return;
      digitalWriteFast(STEP_DIR, LOW);
      curLiftStep--;
    }
    pulse_state = !pulse_state;
    digitalWriteFast(STEP_PULSE, pulse_state);
  }
return;
 
}
uint8_t ppu_report[50];
void RobotDriver::reportPPU()
{
  ppu_report[0]=0xaa;
  ppu_report[1]=0x55;
  ppu_report[2]=0xaa;
  ppu_report[3]=0xF1;
  ppu_report[4]=0xF2;
  ppu_report[5]=0x04;
  ppu_report[6]=0xFF;
  ppu_report[7]=0x00;
  ppu_report[8]=0x00;
  ppu_report[9]=0x00;
  uint8_t byte4 = 0;
  if(bot_mode==MODE_STANDBY){
    byte4+=0x80;
    // Serial.println("standby on");
  }
  else {
    byte4+=0x40;
    // Serial.println("standby off");
  }
  ppu_report[10]=byte4;
  uint16_t crc = gen_crc16(ppu_report,11);
  ppu_report[11]=crc>>8;
  ppu_report[12]=crc&0xff;
  S_COMMAND.write(ppu_report,13);
  // S_COMMAND
}
float RobotDriver::loadParam(String id, float defaultValue=0)
{
  for (unsigned int i=0;i<paramTable.size();i++ )
  {
    if(paramTable[i].paramName.equals(id))return paramTable[i].paraValue;
  }
  RobotParam newParam;
  newParam.paraValue = defaultValue;
  newParam.paramName = id;
  paramTable.push_back(newParam);
  return defaultValue;
}
void RobotDriver::setParam(String id, float value)
{
  paramchanged = true;
  for (unsigned int i=0;i<paramTable.size();i++ )
  {
    if(paramTable[i].paramName.equals(id))
    {
      paramTable[i].paraValue = value;
      loadParams();
      return;
    }
  }
  RobotParam newParam;
  newParam.paraValue = value;
  newParam.paramName = id;
  paramTable.push_back(newParam);
  loadParams();
  
  return ;
}
void RobotDriver::processCommand(String command) {
  // Serial.print(command);
  std::vector<String> tokens = splitString(command,',');
  if (tokens.size() >= 2) {
    if (tokens[1].equals("sync")) {
        sbus.syncLossCount=0;
        // Serial.print("sync");
      }
    else{
      DPRINT("!$Command:"); DPRINT(curTime); DPRINT(command); DPRINT("#");DPRINT("@");
      if (tokens[1].equals("m")&&(tokens.size() >= 4)) {
        float comX = (tokens[2].toFloat());
        float comY = tokens[3].toFloat();
        desX = comX;
        desY = comY;
        gotoMode(MODE_MOVE);
        // DPRINTLN(comX);
        // DPRINTLN(comY);
      }
      if ((tokens[1].equals("set"))&&(tokens.size() >= 4)) {

        String id = (tokens[2]);
        float value = tokens[3].toFloat();
        setParam(id,value);
        // Serial.print(command);
        // DPRINTLN(comX);
        // DPRINTLN(comY);
      }
      
      if (tokens[1].equals("a")) {
        desAngle = (tokens[2].toFloat());
        // float comY = tokens[3].toFloat();
        // desX = comX;
        // desY = comY;
        gotoMode(MODE_ROTATE);
        // DPRINTLN(comX);
        // DPRINTLN(comY);
      }
      if (tokens[1].equals("d")) {
        
        float distance = tokens[2].toFloat()*1000;
        // desAngle = (tokens[2].toFloat());
        // float comY = tokens[3].toFloat();
        desX = desX+distance*sin(desAngle/DEG_RAD);
        desY = desY+distance*cos(desAngle/DEG_RAD);
        gotoMode(MODE_MOVE);
        Serial.print("$!COM,");
        Serial.println(desX);
        Serial.println(desY);
        Serial.print('#');
        // DPRINTLN(comX);
        // DPRINTLN(comY);
      }
      if (tokens[1].equals("rp")) {

        // desAngle += (tokens[2].toFloat())*90;
        // float comY = tokens[3].toFloat();
        // desX = desX+sin(radians(desAngle));
        // desY = desY+cos(radians(desAngle));
        desX=0;
        desY=0;
        botx=0;
        boty=0;
        botangle=0;
        desAngle = 0;
        bot_mode = MODE_STANDBY;
        imu.resetYaw(0);
        curLiftStep = 0;
        // DPRINTLN(comX);
        // DPRINTLN(comY);
      }
      if (tokens[1].equals("lift")) {
        if(tokens[2].toInt()==1)cur_lift_stat = 1;
        if(tokens[2].toInt()==-1)cur_lift_stat = 0;
        newliftComm=cur_lift_stat;
        gotoMode(MODE_LIFT);
        // DPRINT("!$Command:");  DPRINTLN(command);  DPRINT("#");
        
      }
      if (tokens[1].equals("r")) {
        desAngle = (tokens[2].toFloat())*90;
        if(desAngle>180)desAngle-=360;
        gotoMode(MODE_ROTATE);
        // Serial.print("$");
        // Serial.print("COMACK,");
        // Serial.println(command);
      }
      if (tokens[1].equals("s")) {

        // desAngle += (tokens[2].toFloat())*90;
        // float comY = tokens[3].toFloat();
        // desX = desX+sin(radians(desAngle));
        // desY = desY+cos(radians(desAngle));
        // bot_mode = MODE_STANDBY;
        gotoMode(MODE_STANDBY);
        // DPRINTLN(comX);
        // DPRINTLN(comY);
      }
      if (tokens[1].equals("param")) {
          paramchanged=true;
        // desAngle += (tokens[2].toFloat())*90;
        // float comY = tokens[3].toFloat();
        // desX = desX+sin(radians(desAngle));
        // desY = desY+cos(radians(desAngle));
        // bot_mode = MODE_STANDBY;
        // gotoMode(MODE_STANDBY);
        // DPRINTLN(comX);
        // DPRINTLN(comY);
      }
      if (tokens[1].equals("home")) {

        // desAngle += (tokens[2].toFloat())*90;
        // float comY = tokens[3].toFloat();
        desX = 0;//desX+sin(radians(desAngle));
        desY = 0;//desY+cos(radians(desAngle));
        gotoMode(MODE_MOVE);
        // DPRINTLN(comX);
        // DPRINTLN(comY);
      }
    }
    // Serial.print("$");
    
    // Serial.print("COMACK,");
    // Serial.print(desX);
    // Serial.print(",");
    // Serial.print(desY);
    // Serial.println(",");
    S_SENSORS.print("$");
    S_SENSORS.println("COMACK,");
  } else {
    S_SENSORS.print("$");
    S_SENSORS.println("COMERR,");
    
  }
}
String commandString;
unsigned char commandMessage[50];
int commandMessageI=-1;
int commandMessagelen=0;
uint8_t bytein1=0,bytein2=0;
uint16_t commandCode=0;
void RobotDriver::updateCommandBus() {
  while (S_DEBUG.available()) {
    
    uint8_t bytein = S_DEBUG.read();
    if(isPrintable (bytein))
    { 
      
      if (commandString.length() >= COMMAND_LEN_MAX) commandString = "";
      commandString += (char)bytein;
      if (bytein == '\n')  //end of command
      {
        if (commandString.indexOf("$COM")>=0) {
          processCommand(commandString);
        }
        commandString = "";
      }
    }
    
  }
  while(S_COMMAND.available())
    {
      uint8_t bytein = S_COMMAND.read();
      Serial.println(bytein);
      Serial.flush();
      commandMessageI++;
      if(commandMessageI>=50)commandMessageI=50;
      if((bytein==0xaa)&&(bytein1==0x55)&&(bytein2==0xaa))
      {
        commandMessage[0]=bytein2;
        commandMessage[1]=bytein1;
          commandMessageI=2;
      }
      bytein2 = bytein1;
      bytein1 = bytein;
      commandMessage[commandMessageI] = bytein;
      processCommandBytes();
    }
}
void RobotDriver::processCommandBytes()
{
  if((commandMessage[0]==0xaa)&&(commandMessage[1]==0x55)&&(commandMessage[2]==0xaa))
  {
    syncLossCount=0;
    
    Serial.println("binary command");
    if((commandMessage[3]==0xf2)&&(commandMessage[4]==0x00))
    {
      if((commandMessage[5]==0x02)&&(commandMessage[6]==0x00))
      {
          if(commandMessageI>=16)
          {
            uint16_t crc = gen_crc16(commandMessage,15);
            uint16_t real_crc = (commandMessage[15]<<8)+commandMessage[16];
            if(crc==real_crc)sendPPUack(2);
            else {
              DPRINT("!$ERROR CRC:");  DPRINTLN(crc);  DPRINTLN(real_crc); DPRINT("#");
            }
            commandMessageI=0;
            desX = (commandMessage[8]<<8)+commandMessage[9];
            if(desX>32767)desX-=65536;
            desX*=100.0;
            desY = (commandMessage[10]<<8)+commandMessage[11];
            if(desY>32767)desY-=65536;
            desY*=100.0;
            uint8_t action = commandMessage[14]&0x0f;
            uint8_t angle = action&0x03;
            if(angle==0)desAngle=0;
            else if(angle==1)desAngle=90;
            else if(angle==2)desAngle=-90;
            else if(angle==3)desAngle=180;
            newliftComm = (action&0x0c)>>2;
            
            
            if(bot_mode==MODE_STANDBY)
            {
              
              DPRINT("!$Command:"); DPRINTLN("PPU ");DPRINTLN(newliftComm); DPRINTLN(angle);DPRINTLN(desX);DPRINTLN(desY); DPRINT("#");DPRINT("@");S_DEBUG.flush();
              gotoMode(MODE_MOVE);
              S_SENSORS.print("$BINCOMACK,");S_SENSORS.print(action ),S_SENSORS.println(',' );S_SENSORS.flush();
              
            }

            
          }
      }
      else if((commandMessage[5]==0x05)&&(commandMessage[6]==0x00))// map data
      {
        if(commandMessageI>=14)
          {
            uint16_t crc = gen_crc16(commandMessage,13);
            uint16_t real_crc = (commandMessage[13]<<8)+commandMessage[14];
            if(1)sendPPUack(5);
            else {
              DPRINT("!$ERROR CRC:");  DPRINTLN(commandMessage[5]); DPRINTLN(crc);  DPRINTLN(real_crc); DPRINT("#");
            }
            commandMessageI=0;
            int newtagX = (commandMessage[7]<<8)+commandMessage[8];
            if(newtagX>32767)newtagX-=65536;
            int newtagY = (commandMessage[9]<<8)+commandMessage[10];
            if(newtagY>32767)newtagY-=65536;
            int id = (commandMessage[11]<<8)+commandMessage[12];
            addFloorTag(id,newtagX*100,newtagY*100);
            DPRINT("!$Command:"); DPRINTLN("new tag");DPRINTLN(newtagX); DPRINTLN(newtagY);DPRINTLN(id); DPRINT("#");DPRINT("@");S_DEBUG.flush();

            
          }
        
      }
      else if((commandMessage[5]==0x04)&&(commandMessage[6]==0x00))
      {
        reportPPU();
      }
    }
    else commandMessage[0]=0xff;//deny current packet
  }
}
uint8_t ppu_ack[10];
void RobotDriver::sendPPUack(uint8_t commandCode = 0)
{
  ppu_ack[0]=0xaa;
  ppu_ack[1]=0x55;
  ppu_ack[2]=0xaa;
  ppu_ack[3]=0xF1;
  ppu_ack[4]=0xF2;
  ppu_ack[5]=0x05;
  ppu_ack[6]=0xFF;
  uint16_t crc = gen_crc16(ppu_ack,7);
  ppu_ack[7]=crc>>8;
  ppu_ack[8]=crc&0xff;
  S_COMMAND.write(ppu_ack,9);
}

RobotDriver::RobotDriver() {
  S_IMU.begin(921600);    //IMU 1Mbps
  S_SENSORS.begin(921600);  //sensor bus 1Mbps
  S_MOTORS.begin(230400); // motor control bus 0.23Mbps
  S_COMMAND.begin(921600); // command bus 1Mbps
  S_DEBUG.begin(2000000); // debug data 2Mbps
  portIMU = &S_IMU;
  portSenBus = &S_SENSORS;
  portMotor = &S_MOTORS;
  imu.IMU_init(portIMU);
  Serial.println("start");
  //initialize floor map
  // addFloorTag(0, 0,   0);
  addFloorTag(1, 0,   0);
  addFloorTag(2, 0, 1000);
  addFloorTag(3, 0, 2000);
  addFloorTag(4, 1000, 0);
  addFloorTag(5, 1000, 1000);
  addFloorTag(6, 1000, 2000);
  addFloorTag(7, 2000, 0);
  addFloorTag(8, 2000, 1000);
  addFloorTag(9, 2000, 2000);
  addFloorTag(10,3000, 0);
  addFloorTag(11,3000, 1000);
  addFloorTag(12,3000, 2000);
  addFloorTag(13,4000, 0);
  addFloorTag(14,4000, 1000);
  addFloorTag(15,4000, 2000);
  addFloorTag(16,4000, 3000);
  addFloorTag(17,3000, 3000);
  addFloorTag(18,2000, 3000);
  addFloorTag(19,1000, 3000);
  addFloorTag(20,5000, 1000);

  desLiftLevel = liftLevelDown;
  // addFloorTag(100, 0 , 0);
  // addFloorTag(101, 1000, 0);
  // addFloorTag(102, 1000, 0);
  // addFloorTag(103, 1000, 0);
  // addFloorTag(104, 1000, 0);
  pinMode(PIN_OUT_1, OUTPUT);
  pinMode(PIN_OUT_2, OUTPUT);
  pinMode(PIN_OUT_3, OUTPUT);
  pinMode(PIN_OUT_4, OUTPUT);

  pinMode(EMG_STOP, INPUT);
  pinMode(ENC_A1, INPUT);
  pinMode(ENC_B1, INPUT);
  pinMode(ENC_A2, INPUT);
  pinMode(ENC_B2, INPUT);
  pinMode(ENC_A3, INPUT);
  pinMode(ENC_B3, INPUT);
  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  // pinMode(ENC_Z3, INPUT); 
#ifdef SIMULATION
  liftLevel=0;
  liftLevelDown = LIFT_PPR/2.0;
  // liftLevelMaxDefined=true;
  liftLevelMinDefined = true;
  
#endif
  if (imu.isConnected()) {
    Serial.println("IMU connect OK");

  } else {
    Serial.println("IMU connect failed");
    blink(3);
  }
  // DPRINTF("RobotDriver Setup");
  Serial.flush();

  // encoderPos = 0;
  botRotationSpeed = 0.0;
  i_limit_pos = 100;
  pinMode(EMG_STOP, INPUT);
  pinMode(ENC_A1, INPUT);
  pinMode(ENC_B1, INPUT);
  pinMode(ENC_A2, INPUT);
  pinMode(ENC_B2, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A1), encRightInt, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), EncLeftInt, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A3), EncIntLift, RISING);
  desAngle = 0;
  loadParams();
  controlPacket[0] = 0xaa;
  controlPacket[1] = 0x55;
  controlPacket[2] = 0x00;
  initOK = true;

}
void RobotDriver::gotoStandby() {
}
void RobotDriver::posUpdate() {//update robot position, lifter status and angles
  // int dt = curTime - lastPosUpdateMillis;  //check dt, should be 20ms
  // if (dt < DT_POS_UPDATE*1000) return;  //dt minimum limit to 150 millis
  // lastPosUpdateMillis= curTime;
  float liftLevelDistance;
#ifdef SIMULATION

  float distanceRight = desMotSpdL * 30.0;  //(encoderPosLeft-encRighto)*0.6135923151;
  float distanceLeft = desMotSpdR  * 30.0;    //(encPosLeft-encLefto)*0.6135923151;
   
  // liftLevel+=desLiftSpeed*DT_CONTROL/360.0*1400.0;
   liftLevelDistance = liftLevel - liftLevelo;
#else
   liftLevelDistance = liftLevel - liftLevelo;
  //  if(abs((long int)(micros()-EncIntLiftTime))>200000)curSpeedLift2=0;
  float distanceRight = -(encPosRight - encRighto) * 0.65;
  float distanceLeft = -(encPosLeft - encLefto) * 0.65;
#endif
  // Serial.println(curSpeedLift);
  float distance = (distanceLeft + distanceRight) / 2.0;
  curSpeed = distance / 1000.0 / DT_POS_UPDATE;
  curSpeedL = distanceLeft / 1000.0 / DT_POS_UPDATE;
  curSpeedR = distanceRight / 1000.0 / DT_POS_UPDATE;
  curSpeedLift = liftLevelDistance/LIFT_PPR/DT_POS_UPDATE/0.925;//round per Sec
  encRighto = encPosRight;
  encLefto = encPosLeft;
  liftLevelo = liftLevel;
  liftLevelAngle=-(liftLevel-liftLevelDown)*360.0/LIFT_PPR/0.925;
  // while (liftLevelAngle<-180)liftLevelAngle+=360;
  // while (liftLevelAngle>180)liftLevelAngle-=360;
  // liftAngle=botangle+liftLevelAngle;

  // while (liftAngle<-180)liftAngle+=360;
  // while (liftAngle>180)liftAngle-=360;
  
  // botangle += botRotationSpeed*DT_CONTROL;
  // while (botangle >= 180) botangle -= 360;
  // while (botangle < -180) botangle += 360;
  botx += sin((botangle)/DEG_RAD) * distance;
  boty += cos((botangle)/DEG_RAD) * distance;
  float diff = (distanceRight- distanceLeft);
  botRotationSpeed = DEG_RAD*((diff / ( 1000.0 * BASE_LEN)))/DT_POS_UPDATE;
#ifdef SIMULATION
  
  botangle += botRotationSpeed*DT_CONTROL;
#else
  float angleIMU = imu_data.gyroyaw;
  
  if (angleIMU > 180) angleIMU -= 360;
  if (angleIMU < -180) angleIMU += 360;
  botangle = angleIMU;
#endif
// Serial.println(botangle);
  // liftLevel += liftLevel;
}
unsigned long lastDebugTime=0;

void RobotDriver::DebugReport()
{
  // Serial.println(curTime-lastDebugTime);
  
  if((curTime-lastDebugTime)<100)return;
  
  lastDebugTime = curTime;
  // debugCounter=0;
  S_SENSORS.print("$");
  S_SENSORS.print("MCU,");
  // S_SENSORS.println("SYNC");
   S_SENSORS.print(botx); //1
   S_SENSORS.print(',');
    S_SENSORS.print(boty); //2
    S_SENSORS.print(',');
     S_SENSORS.print(botangle); //3
     S_SENSORS.print(',');
     S_SENSORS.print(liftAngle);//4
     S_SENSORS.print(',');
     S_SENSORS.print(curSpeedL);//5
     S_SENSORS.print(',');
     S_SENSORS.print(curSpeedR);//6
     S_SENSORS.print(',');
     S_SENSORS.print(botRotationSpeed);//7
     S_SENSORS.print(',');
     S_SENSORS.print(sbus.tagAngle);//8
     S_SENSORS.print(',');
     S_SENSORS.print(desX);//9
     S_SENSORS.print(',');
     S_SENSORS.print(desY);//10
     S_SENSORS.print(',');
     S_SENSORS.print(lastFloorTagid);//10
     S_SENSORS.print(',');
     S_SENSORS.print(lastFloorYawTagid);//11
     S_SENSORS.print(',');
     S_SENSORS.print(bot_mode);//12
     S_SENSORS.print(',');
     S_SENSORS.print(liftAngleError);//13
     S_SENSORS.print(',');
     S_SENSORS.print(isLiftMaxPos);//14
     S_SENSORS.print(',');
     S_SENSORS.print(curLiftStep);//15
     S_SENSORS.print(',');
     S_SENSORS.print(desLiftStep);//16
     S_SENSORS.print(',');
     S_SENSORS.print(cur_lift_stat);//16
     
     
     S_SENSORS.print(",#\n");

  return;
  debugCounter++;
  if(debugCounter>=3)debugCounter=0;    
  
  DPRINT("!$ curSpeed L R Li Ro:");  DPRINTLN(curSpeedL);  DPRINTLN(curSpeedR);  DPRINTLN(curSpeedLift); DPRINTLN(botRotationSpeed); DPRINT("#");
  DPRINT("!$DesMotSpd RLL:");  DPRINTLN(desMotSpdR);   DPRINTLN(desMotSpdL); DPRINTLN(desLiftSpeed);DPRINTLN(desSpeed);  DPRINT("#");
  DPRINT("!$RotSpd des cur:");     DPRINTLN(desRotSpd); DPRINTLN(imu_data.gyroZ);  DPRINT("#");S_DEBUG.flush();S_DEBUG.print('@');
  if(debugCounter==0){
    DPRINT("!$PID yaw:");       DPRINTLN(Kp_yaw * error_yaw);DPRINTLN(Ki_yaw * integral_yaw);DPRINTLN(Kd_yaw * derivative_yaw);DPRINT("#");
    DPRINT("!$PID pos:");       DPRINTLN(Kp_pos * error_pos);DPRINTLN(Ki_pos * integral_pos);DPRINTLN(Kd_pos * derivative_pos);DPRINT("#");
    DPRINT("!$LiftLevel cur des:");     DPRINTLN(curLiftStep);  DPRINTLN(desLiftLevel);DPRINT("#");
    

  }
  if(debugCounter==1){
    DPRINT("!$dx,dy,dd,da:");  DPRINTLN(desX);  DPRINTLN(desY);  DPRINTLN(desDistance);  DPRINTLN(desAngle);  DPRINT("#");
    DPRINT("!$x,y,ba,la:");  DPRINTLN(botx);  DPRINTLN(boty);  DPRINTLN(botangle); DPRINTLN(liftAngle);  DPRINT("#");
    DPRINT("!$floorMap:");  DPRINTLN(floorMap.size()); DPRINTLN(lastFloorTagid);  DPRINT("#");
    
  }
  if(debugCounter==2){
  DPRINT("!$GyroYaw:");  DPRINTLN(imu_data.gyroyaw);  DPRINT("#");
  DPRINT("!$PIDerr YPL:");     DPRINTLN(error_yaw);  DPRINTLN(error_pos); DPRINTLN(liftAngleError); DPRINT("#");  
  DPRINT("!$curtime cursyns:");     DPRINTLN(curTime);  DPRINTLN(syncLossCount); DPRINT("#");  
  
//  sendSyncPacket() ;
  }
  S_DEBUG.print('@');
  S_DEBUG.flush();

}
void RobotDriver::controlLoop()// high frequency(>1khz) controll loop
{
  liftStabilize(); 
  stepOutput(1);
  int dt = curTime - lastLoopMillis;  //check dt, should be 20ms
  
  if (dt < 10) return;  //dt minimum limit to 10 millis, limit loop rate to 100hz
  
  lastLoopMillis = curTime;
  int times500ms = curTime/2000;
  sbus.syncLossCount++;
  if((sbus.syncLossCount>100)&&(bot_mode!=MODE_STANDBY))
  {
    gotoMode(MODE_STANDBY);
  }
  if(times500ms!=lastSyncSec)
  {
    sendSyncPacket();
    reportPPU();
    lastSyncSec = times500ms;
  }
  posUpdate();
  DebugReport();
  checkMinMax();

  switch (bot_mode)// call loop function based on robot mode
  {
  case MODE_STANDBY:
    loopStandby();
    break;
  case MODE_MOVE:
    loopMove();
    break;
  case MODE_ROTATE:
    loopRotate();
    
    break;
  case MODE_LIFT:
    loopLift();
    break;
  default:
    break;
  }
  if(bot_mode==MODE_STANDBY)
  {
    int led_circle = (curTime%4000)/2;
    analogWrite(PIN_OUT_3,(sin(led_circle/159.2)+1)*100.0);
    // analogWrite(PIN_OUT_2,(sin(3.14+led_circle/159.2)+1)*100.0);
  }
  else
  {
    int led_circle = (curTime%1000);//0 to 1000
    int led_step = led_circle/100;//0 to 10
    int led_on = led_step%2;
    if(led_step<(bot_mode*2))
    {
      analogWrite(PIN_OUT_3,led_on*200);//analogWrite(PIN_OUT_2,led_on*200);
      }
    else {analogWrite(PIN_OUT_3,LOW);}//analogWrite(PIN_OUT_2,LOW);}
  }
}
void RobotDriver::loopMove() {// loop when robot is executing a motion command

  //error calculation
  float dx = desX - botx;
  float dy = desY - boty;

  desDistance = sqrt(dx * dx + dy * dy);
  desBearing = ConvXYtoAngle(dx, dy);
  yaw_PID =  calcPIDyaw(desBearing);
  // Serial.println(yaw_PID);
  //PID speed
  error_pos = desDistance * cos((error_yaw)/DEG_RAD);
  float error_pos_perpendic = desDistance * sin((error_yaw)/DEG_RAD);
  // if (abs(error_yaw) > 90) {
  //   error_yaw-=180;
  //   while (error_yaw<-180)error_yaw+=360;
  // }
  // if (abs(error_yaw) > 10) {
  //   error_pos/=(abs(error_yaw)/10.0);
  // }
  
  if((abs(error_pos) < 40))
  {
    stillCount++;
    if(stillCount>100)  {
      if(cur_lift_stat!=newliftComm)
        {
        cur_lift_stat=newliftComm;
        gotoMode(MODE_LIFT);
        }
      else gotoMode(MODE_ROTATE);
    }
    integral_pos=0;
    error_pos_prev=0;
    error_pos=0;
    return;
  }
  else
    {
    if(stillCount>0)stillCount--;
    }
  pos_PID =  calcPIDPos(error_pos);
  desSpeed = (desMotSpdL+desMotSpdR)/2.0;
  if (abs(error_yaw) > 10) desSpeed*=5.0/abs(error_yaw);
  float newdesSpeed = constrainVal(pos_PID, -maxBotSpeed, maxBotSpeed);
  float acc = newdesSpeed- desSpeed;
  if(acc*desSpeed<0)acc = constrainVal(acc, -maxBotAcc*0.8, maxBotAcc*0.8 );//slow down
  else acc = constrainVal(acc, -maxBotAcc*0.5 , maxBotAcc *0.5);
  desSpeed += acc;
  float rotationReductionRatio = (maxBotSpeed-abs(desSpeed))/maxBotSpeed;//high desSpeed less rotation speed
  if(rotationReductionRatio<0.2)rotationReductionRatio=0.2;
  if(abs(error_pos_perpendic)<80)rotationReductionRatio*=(abs(error_pos_perpendic)/80.0);//less distance less rotation
  // 
  desRotSpd =  constrainVal(yaw_PID, -maxBotRotSpd, maxBotRotSpd);
  desRotSpd*= rotationReductionRatio;  
  setSpeedRight(desSpeed- desRotSpd * BASE_LEN / 2.0);
  setSpeedLeft(desSpeed+desRotSpd * BASE_LEN / 2.0 );  
  // liftStabilize();

}
void RobotDriver::update() {//high speed update to read sensor bus
  if (!initOK) return;
   curTime = millis();
     
  // readSensBus
  while (portSenBus->available()) {
    unsigned char inputByte = portSenBus->read();
    int result = sbus.Input(inputByte);
    if(result==1)//camera 1
    {
      
      FloorTag mapPoint = getFloorTag(sbus.tagID);
      if(mapPoint.id>0)
      {
        
        float tagDistance = 0;//sqrt(sbus.tagX*sbus.tagX+sbus.tagY*sbus.tagY);
        float tagBearing = 0;
        ConvXYToPolar(sbus.tagX,sbus.tagY,&tagBearing,&tagDistance);
        float bearingFromTag = tagBearing+sbus.tagAngle-180;
        float dx = tagDistance*sin(bearingFromTag/DEG_RAD);
        float dy = tagDistance*cos(bearingFromTag/DEG_RAD);
        botx = mapPoint.x+dx;
        boty = mapPoint.y+dy;
        float yawDiff =0;
        if((tagDistance<80)&&(abs(botRotationSpeed)<10))
        {
          
          yawDiff = (sbus.tagAngle-imu_data.gyroyaw);
          while(yawDiff>180)yawDiff-=360;
          while(yawDiff<-180)yawDiff+=360;
          // yawDiff/=5.0;//smooth the change
          if(abs(curSpeedL)+abs(curSpeedR)<0.1) 
          {
            imu.resetYaw(imu_data.gyroyaw+yawDiff);
            lastFloorYawTagid= mapPoint.id;
          }
        }
        if(lastFloorTagid!=mapPoint.id)
        {
          // DPRINT("!$Command:"); DPRINTLN("tagDetect");DPRINTLN(lastFloorTagid);DPRINTLN(mapPoint.id);DPRINTLN(dx); DPRINTLN(dy); DPRINTLN(yawDiff);DPRINT("#");DPRINT("@");S_DEBUG.flush();
        }
        lastFloorTagid = mapPoint.id;
        
      }
      // Serial.println(bearingFromTag);
      // botx = sbus.tagX;
      // boty = sbus.tagY;
      S_SENSORS.print("$CAM1ACK,");
      S_SENSORS.println(lastFloorTagid);
      
    }
    else if(result==2)//command
    {
      processCommand(sbus.commandBuff);
    }
  }
  imu.updateData();
  imu_data = imu.getMeasurement();
  
  if (imu.isConnected() == false) this->isActive = false;
  //read Motor report
  while (portMotor->available()) {
    unsigned char inputByte = portMotor->read();
    // Serial.print(char(inputByte));
    processMotorReport(inputByte);
  }
  //read imcoming command
  updateCommandBus();
  controlLoop();
  
}
 
void RobotDriver::gotoMode(int mode)// change the action mode of robot
{
  resetIntegral();
  bot_mode = mode;
  integral_pos=0;
  error_pos_prev=0;
  liftAngleErroro = 0;
  resetDesRotSpd();
  error_pos=0;
  error_yaw_prev=0;
  integral_yaw=0;
  error_yaw=0;
  stillCount=0;
}
float RobotDriver::calcPIDPos(float epos)//PID calculation for position loop
{
  error_pos = epos;
  if(abs(error_pos)<150)setIntegralPos( error_pos *DT_CONTROL);
  else setIntegralPos(0);
  integral_pos = getIntegralPos();
  // integral_pos += error_pos * DT_CONTROL;
  // integral_pos = constrainVal(integral_pos, -i_limit_pos, i_limit_pos);  //saturate integrator to prevent unsafe buildup
  if(error_pos_prev==0)error_pos_prev=error_pos;
  derivative_pos = (error_pos - error_pos_prev) / DT_CONTROL;
  float pid_out = (Kp_pos * error_pos + Ki_pos * integral_pos + Kd_pos * derivative_pos)/1000.0;  //scaled by .01 to bring within -1 to 1 range
  error_pos_prev = error_pos;
  return pid_out;
}

float RobotDriver::calcPIDyaw(float targetAngle)//PID calculation for yaw loop
{
  error_yaw = targetAngle - botangle;
  
  while (error_yaw > 180) error_yaw -= 360;
  while (error_yaw < -180) error_yaw += 360;
  int predictedLiftStep = (curLiftStep+STEP_PPR/360* error_yaw);
  if(predictedLiftStep>(desLiftStep+STEP_PPR*0.55))error_yaw-=360;
  if(predictedLiftStep<(desLiftStep-STEP_PPR*0.55))error_yaw+=360;
  float scaled_error_yaw=constrain(error_yaw, -60, 60);
  if(abs(error_yaw)<5)  setIntegralYaw( scaled_error_yaw);
  else   setIntegralYaw( 0);

  integral_yaw = getIntegralYaw();
  // integral_yaw = constrainVal(integral_yaw, -scaledIlimit, scaledIlimit);  //saturate integrator to prevent unsafe buildup
  if(error_yaw_prev==0)error_yaw_prev=scaled_error_yaw;
  derivative_yaw = (scaled_error_yaw - error_yaw_prev) / DT_CONTROL;
  float pid_out = 0.02*(Kp_yaw * scaled_error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw);  
  error_yaw_prev = scaled_error_yaw;
  return pid_out;
}
void RobotDriver::setSpeedRight(float value)// set the speed of right wheel 
{
  value = constrainVal(value, -maxBotSpeed, maxBotSpeed);
  float acc = value- desMotSpdR;//todo: use curSpeedR for better accuracy
  // if(acc*desMotSpdR<0)acc = constrainVal(acc, -maxBotAcc*3, maxBotAcc*3 );
  // else 
  acc = constrainVal(acc, -maxBotAcc , maxBotAcc );
  desMotSpdR += acc;
  sendControlPacket(1, desMotSpdR/MAX_MOTION_SPEED, 0);
}
void RobotDriver::setSpeedLeft(float value)// set the speed of left wheel 
{
  value = constrainVal(value, -maxBotSpeed, maxBotSpeed);
  float acc = value- desMotSpdL;//todo: use curSpeedL for better accuracy
  // if(acc*desMotSpdL<0)acc = constrainVal(acc, -maxBotAcc*3, maxBotAcc*3);
  // else 
  acc = constrainVal(acc, -maxBotAcc , maxBotAcc );
  desMotSpdL += acc;
  sendControlPacket(2, desMotSpdL/MAX_MOTION_SPEED, 0);
}
void RobotDriver::loopRotate()// control loop when robot is changing drirection
{
  // DPRINT("!$bot_mode:");      DPRINTLN(bot_mode); DPRINTLN(desAngle); DPRINTLN(desX);DPRINTLN(desY);DPRINT("#");Serial.flush();
  yaw_PID = calcPIDyaw(desAngle);
  if (abs(error_yaw) < 2)
  {
    stillCount+=1;
    if(stillCount>100)  
    {
      if(cur_lift_stat!=newliftComm)
      {
        cur_lift_stat=newliftComm;
        gotoMode(MODE_LIFT);
      
      }else  gotoMode(MODE_STANDBY);
    }
    integral_pos=0;
    error_pos_prev=0;
    error_pos=0;
  }
  else if (abs(error_yaw) < 1.2)
  {
    stillCount+=2;
    if(stillCount>100) {
    if(cur_lift_stat!=newliftComm)
      {
        cur_lift_stat=newliftComm;
        gotoMode(MODE_LIFT);
      
      }else  gotoMode(MODE_STANDBY);
    }
    integral_pos=0;
    error_pos_prev=0;
    error_pos=0;
  }
  else {
    if(stillCount>0)stillCount--;
  }

  float newdesRotSpd = yaw_PID;  
  float desRotAcc = newdesRotSpd-desRotSpd;
  desRotAcc = constrainVal(desRotAcc, -0.02, 0.02);
  desRotSpd+=desRotAcc;
  desRotSpd = constrainVal(desRotSpd, -maxBotRotSpd, maxBotRotSpd);
  setSpeedRight(- desRotSpd * BASE_LEN / 2.0);
  setSpeedLeft(desRotSpd * BASE_LEN / 2.0);  
  // desliftAngle = 0;
  // liftStabilize();
  
}
float liftAngleErrori=0;
void RobotDriver::liftStabilize()//stabilize lift angle with microstep motor driver
{

  liftAngle = -curLiftStep*360.0/STEP_PPR;
  if(1)
  {
    float bu_do_do = lift_gear_lash*desRotSpd*3;
    liftAngle+=bu_do_do;

  }

  // Serial.print(pre);
  // Serial.print(" ");
  // // Serial.print(0.5);
  // Serial.println(liftAngle);
  float liftAngleErroro=liftAngleError;
  liftAngleError=(liftAngle+botangle);
  
  while (liftAngleError>180)liftAngleError-=360;
  while (liftAngleError<-180)liftAngleError+=360;
  // Serial.println(liftAngleError);
  // liftAngleError
  // desLiftStep = botangle/360.0*STEP_PPR;
  // int errLiftStep = liftAngleError*STEP_PPR/360.0;
  // Serial.println(errLiftStep);
  float liftAngleErrord = 1000*(liftAngleError-liftAngleErroro);
  liftAngleErrori+=liftAngleError;
  liftAngleErrori = constrainVal(liftAngleErrori,-1,1);
  float liftPID = 20*(Kp_lift*liftAngleError+Ki_lift*liftAngleErrori+Kd_lift*liftAngleErrord);

  // Serial.print(Kp_lift*liftAngleError);
  // Serial.print(" ");
  // Serial.print(Ki_lift*liftAngleErrori);
  // Serial.print(" ");
  // Serial.print(Kd_lift*liftAngleErrord);
  // Serial.print(" ");
  float newdesLiftSpeed = liftPID;
  float liftAcc = newdesLiftSpeed-desLiftSpeed;
  // liftAcc = constrainVal(liftAcc, -0.0004, 0.0004);
  desLiftSpeed+=liftAcc;
  // Serial.println(desLiftSpeed);
  // desLiftSpeed = constrainVal(desLiftSpeed, -0.13, 0.13);
  stepFreq=abs(STEP_PPR*desLiftSpeed);
  pulse_period_us = 1000000/(stepFreq+1);
  // Serial.println(desLiftSpeed);
  // stepFreq = (stepFreq*0.6+newstepFreq*0.4);
  // if(stepFreq>2000)Serial.println(stepFreq);
  
    // liftAngleError = desliftAngle - liftAngle;
    // while (liftAngleError<-180)liftAngleError+=360;
    // while (liftAngleError>180)liftAngleError-=360;
    // float integral_lift=0;
    // if(liftAngleError<10){
    //   setIntegralLift( liftAngleError *DT_CONTROL);
    // integral_lift = getIntegralLift();
    // }
    // float errorDiff = 0;
    // if(liftAngleErroro!=0)errorDiff = liftAngleError-liftAngleErroro;
    // liftAngleErroro = liftAngleError;
    // float predictionLiftSpeed = getdesRotSpdDelay(desRotSpd,liftStabDelay);
    // float newdesLiftSpeed =  0.1*predictionLiftSpeed*Ks_lift - Kp_lift*liftAngleError/90.0 - Kd_lift * curSpeedLift2 - integral_lift*Ki_lift ;//+botRotationSpeed/40.0;//+= dliftSpeed;
    // float liftAcc = newdesLiftSpeed-desLiftSpeed;
    // liftAcc = constrainVal(liftAcc, -maxLiftAcc, maxLiftAcc);
    // desLiftSpeed+=liftAcc;
    // desLiftSpeed = constrainVal(desLiftSpeed, -0.1, 0.1);
    // sendControlPacket(3, desLiftSpeed, 0);
}
void RobotDriver::liftStabilizeBLV()// stabilize lift angle with BLHM motor driver
{
    // liftAngleError = desliftAngle - liftAngle;
    // while (liftAngleError<-180)liftAngleError+=360;
    // while (liftAngleError>180)liftAngleError-=360;
    // float integral_lift=0;
    // if(liftAngleError<10){
    //   setIntegralLift( liftAngleError *DT_CONTROL);
    //   integral_lift = getIntegralLift();
    // }
    // float errorDiff = 0;
    // if(liftAngleErroro!=0)errorDiff = liftAngleError-liftAngleErroro;
    // liftAngleErroro = liftAngleError;
    // float predictionLiftSpeed = getdesRotSpdDelay(desRotSpd,liftStabDelay);
    // float newdesLiftSpeed =  0.1*predictionLiftSpeed*Ks_lift - Kp_lift*liftAngleError/90.0 - Kd_lift * curSpeedLift2 - integral_lift*Ki_lift ;//+botRotationSpeed/40.0;//+= dliftSpeed;
    float newdesLiftSpeed = -imu_data.gyroZ*1.1;
    float liftAcc = newdesLiftSpeed-desLiftSpeed;
    // liftAcc = constrainVal(liftAcc, -maxLiftAcc, maxLiftAcc);
    desLiftSpeed+=liftAcc;
    desLiftSpeed = constrainVal(desLiftSpeed, -0.999, 0.999);
    sendControlPacket(3, desLiftSpeed, 0);
    Serial.println(desLiftSpeed);
}
float descrease(float input)
{
    if(input>0.01)return input-0.01;
    if(input<-0.01)return input+0.01;
    return 0;
}
void RobotDriver::loopStandby()// control loop on standby mode
{
  // posUpdate();
  

  setSpeedLeft(descrease(desMotSpdL));
  setSpeedRight(descrease(desMotSpdR));
  // desLiftSpeed = descrease(desLiftSpeed);
  // sendControlPacket(3, desLiftSpeed, 0);
  // liftStabilizeBLV();
}
void RobotDriver::sendSyncPacket() {
  if(paramchanged){
    for(unsigned int i =0;i<paramTable.size();i++)
    {
      S_SENSORS.print("$MCUPARAM,"); 
      S_SENSORS.print(paramTable[i].paramName);
      S_SENSORS.print(",");
      S_SENSORS.print(paramTable[i].paraValue);
      S_SENSORS.print(",");
      S_SENSORS.print("#");S_SENSORS.print('\n');S_SENSORS.flush();
    }
    for(unsigned int i =paramTable.size()-1;i>0;i--)
    {
      S_SENSORS.print("$MCUPARAM,"); 
      S_SENSORS.print(paramTable[i].paramName);
      S_SENSORS.print(",");
      S_SENSORS.print(paramTable[i].paraValue);
      S_SENSORS.print(",");
      S_SENSORS.print("#");S_SENSORS.print('\n');S_SENSORS.flush();
    }
    paramchanged=false;
  }
  return;
  DPRINT("!$lastSyncSec:");  DPRINTLN(lastSyncSec);  DPRINT("#");
  DPRINT("!$paramTable.size:");  DPRINTLN(paramTable.size());  DPRINT("#");
  DPRINT("!$GyroConnect:");  DPRINTLN(imu.isConnected());  DPRINT("#");
  DPRINT('@');S_DEBUG.flush();
  DPRINT("!$Lift Status:");   DPRINTLN(curLiftStep);DPRINTLN(minLiftStep);  DPRINTLN(desLiftStep); DPRINTLN(desLiftSpeed);   DPRINT("#");
  DPRINT("!$Motor Fail:");    DPRINTLN(M1ComTime);   DPRINTLN(M2ComTime);   DPRINTLN(M3ComTime);   DPRINT("#");
  DPRINT("!$bot_mode:");      DPRINTLN(bot_mode); DPRINTLN(stillCount); DPRINT("#");
  DPRINT('@');S_DEBUG.flush();
  // uint globalmsec = curTime;
  // controlPacket[2] = 0x00;
  // controlPacket[3] = (0xff) & (globalmsec >> 16);
  // controlPacket[4] = (0xff) & (globalmsec >> 8);
  // controlPacket[5] = (0xff) & globalmsec;
  // controlPacket[6] = calcCS8(controlPacket, 6);
  // portMotor->write(controlPacket, 7);
  
}
void RobotDriver::loadParams()//load robot parameters from memory
{
  Kp_yaw = loadParam("Kp_yaw",8) ;   //Yaw P-gain
  Ki_yaw = loadParam("Ki_yaw",1.0);   //Yaw I-gain
  Kd_yaw = loadParam("Kd_yaw",1.5);  //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
  Kp_pos = loadParam("Kp_pos",2.2);  //Yaw P-gain
  Ki_pos = loadParam("Ki_pos",11.0);  //Yaw I-gain
  Kd_pos = loadParam("Kd_pos",0.2);  //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
  Kp_lift = loadParam("Kp_lift",1.0);  
  Ki_lift = loadParam("Ki_lift",0.0);  
  Kd_lift = loadParam("Kd_lift",0.05);  
  Ks_lift = loadParam("Ks_lift",0.6);  
  maxBotSpeed = loadParam("maxBotSpeed",0.4);
  maxBotRotSpd = loadParam("maxBotRotSpd",0.8);
  maxBotAcc = loadParam ("maxBotAcc",5.0)/1000.0;
  liftStabDelay = loadParam ("liftStabDelay",15);
  maxLiftAcc = loadParam ("maxLiftAcc",2)/1000.0;
  lift_gear_lash = loadParam ("lift_gear_lash",2);
  paramchanged=true;
}
void RobotDriver::sendControlPacket(uint8_t id, float speed, uint8_t mode) {//control command for motor boards
  // control left motor
  controlPacket[2] = id;
  if (id == 1) M1ComTime  = curTime;
  if (id == 2) M2ComTime  = curTime;
  if (id == 3) M3ComTime  = curTime;
  if (speed > 0) controlPacket[4] = (0xAB);
  else controlPacket[4] = (0xBA);
  speed=constrainVal(speed,-1.0,1.0);
  controlPacket[3] = ((unsigned char)(abs(speed) * 255));
  controlPacket[5] = mode;  //Operation Mode
  controlPacket[6] = calcCS8(controlPacket, 6);
  portMotor->write(controlPacket, 7);
  portMotor->flush();
  // delayMicroseconds(20);
  // DPRINTLN(millis());
}
void RobotDriver::checkMinMax()
{
  isLiftMinPos = (digitalRead(19));
  isLiftMaxPos = (digitalRead(18));
  if (isLiftMinPos) 
  {
    liftLevelMinDefined = true;
    minLiftStep = curLiftStep;
    desLiftStep = minLiftStep+STEP_PPR/2;
    // liftComm=0;
    DPRINT("!$Command:"); DPRINT(curTime); DPRINT("liftLevelMinDefined:"); DPRINT(minLiftStep);DPRINT("#");DPRINT("@");
    // desLiftLevel=liftLevelDown;
    // gotoMode(MODE_LIFT);
  }
  // if(isLiftMaxPos)
  // {
  //   minLiftStep=curLiftStep-LIFT_PPR*8;
  //   desLiftStep = minLiftStep+STEP_PPR*7;
  // }
}
uint8_t reportPacket[50];
int MotorReportBuffID = 0;
void RobotDriver::processMotorReport(uint8_t bytein) {//read report packets from motor boards
  // DPRINTLN(bytein);
  uint8_t lastByte = reportPacket[MotorReportBuffID];
  if ((bytein == 0x55) && (lastByte == 0xaa)) {
    MotorReportBuffID = 1;
    reportPacket[0] = 0xaa;
    reportPacket[1] = 0x55;
  } else {
    MotorReportBuffID++;
    if (MotorReportBuffID >= 49) MotorReportBuffID = 0;
    reportPacket[MotorReportBuffID] = bytein;
  }
  
  if (MotorReportBuffID > 7) {
    if (reportPacket[1] == 0x55)
      if (reportPacket[0] == 0xaa) {
        if (reportPacket[2] == 0x83)  //lift motor report
        {
          M3ComTime = 0;
          uint8_t cs = calcCS8(reportPacket, 7);
          if (cs == reportPacket[7]) {
            isLiftMinPos = (reportPacket[5] == 0);
            isLiftMaxPos = (reportPacket[6] == 0);
            if (isLiftMinPos) 
            {
              liftLevelMinDefined = true;
              minLiftStep = curLiftStep;
              desLiftStep = minLiftStep+STEP_PPR/2;
              // liftComm=0;
              DPRINT("!$Command:"); DPRINT(curTime); DPRINT("liftLevelMinDefined:"); DPRINT(minLiftStep);DPRINT("#");DPRINT("@");
              // desLiftLevel=liftLevelDown;
              // gotoMode(MODE_LIFT);
            }
            if(isLiftMaxPos)
            {
              minLiftStep=curLiftStep-LIFT_PPR*8;
              desLiftStep = minLiftStep+STEP_PPR*7;
            }
            
            // reportCount++;
          } else {
            DPRINT("!$CSfail:");
            DPRINTLN(cs);
            DPRINTLN(reportPacket[7]);
            DPRINT("#");
          }
        }
        if (reportPacket[2] == 0x82)  //lift motor report
        {
          M2DelayTime = curTime-M2ComTime;
        }
        if (reportPacket[2] == 0x81)  //lift motor report
        {
          M1DelayTime = curTime-M1ComTime;
        }
      }
    // MotorReportBuffID=0;
  }
  // if (MotorReportBuffID >= 19)
  //   if (reportPacket[0] == 0xaa)
  //     if (reportPacket[1] == 0x55)
  //       if (reportPacket[2] == 0xf1)
  //         if (reportPacket[4] == 0xff)
  //           if (reportPacket[5] == 0x03) {
  //             // DPRINTLN("\npacketOK");
  //             int x = (reportPacket[6] << 24) + (reportPacket[7] << 16) + (reportPacket[8] << 8) + reportPacket[9];
  //             int y = (reportPacket[10] << 24) + (reportPacket[11] << 16) + (reportPacket[12] << 8) + reportPacket[13];
  //             float angle = (reportPacket[14] << 8) + reportPacket[15];
  //             angle /= 10.0;

  //             botangle = angle;
  //           }
}
void RobotDriver::motorUpdate()
{
  //not implemented
    // stepOutput(1);
}
void RobotDriver::loopLift()// control loop when robot is on lifting mode 
{

  switch (cur_lift_stat)
  {
  case 0:
    desLiftStep = minLiftStep+STEP_PPR/2;
    break;
  case 1:
    desLiftStep = minLiftStep+STEP_PPR*7;
    break;
  case 2:
    desLiftStep = minLiftStep+STEP_PPR/2;
    break;
  case 3:
    desLiftStep = minLiftStep+STEP_PPR*7;
    break;
  default:
    break;
  }
  
  float desLiftStepError = desLiftStep-curLiftStep;
  if(liftLevelMinDefined)
  {
    if(abs(desLiftStepError)<(STEP_PPR/4))stillCount++;
    else stillCount=0;
    if(abs(desLiftStepError)<(STEP_PPR/4)&&stillCount>200)gotoMode(MODE_ROTATE);
    float newdesRotSpd = desLiftStepError/1000.0;  
    float desRotAcc = newdesRotSpd-desRotSpd;
    desRotAcc = constrainVal(desRotAcc, -0.02, 0.02);
    desRotSpd+=desRotAcc;
    desRotSpd = constrainVal(desRotSpd, -maxBotRotSpd, maxBotRotSpd);
    setSpeedRight(- desRotSpd * BASE_LEN / 2.0);
    setSpeedLeft(desRotSpd * BASE_LEN / 2.0);  
  }
  else
  {
    float newdesRotSpd = -1;  
  float desRotAcc = newdesRotSpd-desRotSpd;
  desRotAcc = constrainVal(desRotAcc, -0.02, 0.02);
  desRotSpd+=desRotAcc;
  desRotSpd = constrainVal(desRotSpd, -maxBotRotSpd, maxBotRotSpd);
  setSpeedRight(- desRotSpd * BASE_LEN / 2.0);
  setSpeedLeft(desRotSpd * BASE_LEN / 2.0);  
  }
  
  // liftStabilize();
  return;
  // if (liftLevelMinDefined && liftLevelMaxDefined)
  //  sendControlPacket(3, desLiftSpeed, 0);
  

  // // float desLiftLevelError = desLiftLevel-liftLevel;
  // float newdesLiftSpeed = desLiftLevelError/400.0;
  // float liftAcc = newdesLiftSpeed-desLiftSpeed;
  // liftAcc = constrainVal(liftAcc, -maxLiftAcc, maxLiftAcc);
  // desLiftSpeed+=liftAcc;
  // desLiftSpeed = constrainVal(desLiftSpeed, -0.08, 0.08);
  // sendControlPacket(3, desLiftSpeed, 0);
  // return;
  // desAngle = -liftLevelAngle;
  //   while (desAngle<-180)desAngle+=360;
  // while (desAngle>180)desAngle-=360;
  // yaw_PID = calcPIDyaw(desAngle);
  // desRotSpd = yaw_PID;  
  // desRotSpd = constrainVal(desRotSpd, -maxBotRotSpd*2, maxBotRotSpd*2);
  // setSpeedRight(- desRotSpd * BASE_LEN / 2.0);
  // setSpeedLeft(desRotSpd * BASE_LEN / 2.0);  
  // if(abs(desLiftLevelError)<10)stillCount++;
  // else stillCount=0;
  // if(abs(desLiftLevelError)<10&&stillCount>100)gotoMode(MODE_STANDBY);
  // // curSpeed
  // desRotSpd = desLiftSpeed*9.4;//botRotationSpeed*9.2;
  // // desRotSpd=desRotSpd-liftAngle/30.0;
  // desRotSpd = constrainVal(desRotSpd, -1.3, 1.3);
  // float acc = 0 - desRotSpd * BASE_LEN / 2.0 - desMotSpdR;
  // acc = constrainVal(acc, -maxBotAcc , maxBotAcc );
  // desMotSpdR += acc;
  // desMotSpdR = constrainVal(desMotSpdR, -1.0, 1.0);
  // acc = 0 + desRotSpd * BASE_LEN / 2.0 - desMotSpdL;
  // acc = constrainVal(acc, -maxBotAcc , maxBotAcc );
  // desMotSpdL += acc;
  // // desMotSpdL*=1.08;
  // desMotSpdL = constrainVal(desMotSpdL, -1.0, 1.0);
  
  // // float errorLift = desLiftLevel-liftLevel;
  // // float newdesLiftSpeed = botRotationSpeed/4.0;
  // // float dliftSpeed = newdesLiftSpeed-desLiftSpeed;
  // // dliftSpeed = constrainVal(dliftSpeed,-0.02/DT_CONTROL,0.02/DT_CONTROL);
  // // desLiftSpeed = botRotationSpeed/DEG_RAD/7.8;//+= dliftSpeed;
  // sendControlPacket(1, desMotSpdR, 0);
  // sendControlPacket(2, desMotSpdL, 0);
}

