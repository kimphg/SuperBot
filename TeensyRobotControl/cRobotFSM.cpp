// #include "usb_serial.h"
// #include "wiring.h"
// #include "core_pins.h"
#include <stdint.h>

// #include "usb_serial.h"
#include <math.h>

#include "cRobotFSM.h"
long int encPosRight = 0;
long int encPosLeft = 0;
long int encRighto = 0;
long int encLefto = 0;
long int liftLevel = 0;
long int liftLevelo = 0;
// long int reportCount = 0;
bool liftLevelMinDefined = false;
bool liftLevelInitOK = false;
int desLiftLevel = 0;
int M1Fail = 0, M2Fail = 0, M3Fail = 0;
std::vector<RobotParam> paramTable;
std::vector<FloorTag> floorMap;
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
void EncIntLift() {
  if (digitalRead(ENC_B3) > 0) {
    liftLevel++;
    // if(liftLevel>2000)liftLevelInitOK = true;
  } else liftLevel--;
  if (liftLevel < 0) liftLevel = 0;
  // if(liftLevel>MAX_LIFT_H)liftLevel=MAX_LIFT_H;
}
float constrainVal(float input, float min, float max) {
  if (input < min) return min;
  if (input > max) return max;
  return input;
}
void addFloorTag(int id,int x,int y)
{
  FloorTag newfloorTag;
  newfloorTag.id=id;
  newfloorTag.x = x;
  newfloorTag.y = y;
  floorMap.push_back(newfloorTag);
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
  if(bot_mode==MODE_STANDBY)byte4+=0x80;
  else byte4+=0x40;
  ppu_report[10]=byte4;
  uint16_t crc = gen_crc16(ppu_report,11);
  ppu_report[11]=crc>>8;
  ppu_report[12]=crc&0xff;
  S_COMMAND.write(ppu_report,13);
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
    if (tokens[1].indexOf("sync")>=0) {
        syncLossCount=0;
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
        // Serial.print("$!COM,");
        // Serial.println(desX);
        // Serial.println(desY);
        // Serial.print('#');
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
        // DPRINTLN(comX);
        // DPRINTLN(comY);
      }
      if (tokens[1].equals("lift")) {
        if(tokens[2].toInt()==1)liftComm = 1;
        if(tokens[2].toInt()==-1)liftComm = 0;
        gotoMode(MODE_LIFT);
        // DPRINT("!$Command:");  DPRINTLN(command);  DPRINT("#");
        
      }
      if (tokens[1].equals("r")) {
        desAngle += (tokens[2].toFloat())*90;
        gotoMode(MODE_ROTATE);
      }
      if (tokens[1].equals("s")) {

        // desAngle += (tokens[2].toFloat())*90;
        // float comY = tokens[3].toFloat();
        // desX = desX+sin(radians(desAngle));
        // desY = desY+cos(radians(desAngle));
        bot_mode = MODE_STANDBY;
        // DPRINTLN(comX);
        // DPRINTLN(comY);
      }
    }
  } else {
    DPRINTLNF("Command too short");
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
            liftComm = (action&0x0c)>>2;
            DPRINT("!$PPU_COMAND XYLA:");  DPRINTLN(desX); DPRINTLN(desY);DPRINTLN(liftComm);DPRINTLN(angle); DPRINT("#");
            if(bot_mode==MODE_STANDBY)gotoMode(MODE_MOVE);
            
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
  S_IMU.begin(921600);    //IMU
  S_SENSORS.begin(921600);  //sens bus
  S_MOTORS.begin(230400);
  S_COMMAND.begin(921600);
  S_DEBUG.begin(2000000);
  portIMU = &S_IMU;
  portSenBus = &S_SENSORS;
  portMotor = &S_MOTORS;
  imu.IMU_init(portIMU);
  Serial.println("start");
  desLiftLevel = liftLevelDown;
  addFloorTag(100, 0 , 0);
  addFloorTag(101, 1000, 0);
  addFloorTag(102, 1000, 0);
  addFloorTag(103, 1000, 0);
  addFloorTag(104, 1000, 0);
#ifdef SIMULATION
  liftLevel=0;
  liftLevelDown = LIFT_PPR/4.0;
  liftLevelMaxDefined=true;
  liftLevelMinDefined = true;
  
#endif
  if (imu.getIsConnected()) {
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
void RobotDriver::posUpdate() {
  // int dt = curTime - lastPosUpdateMillis;  //check dt, should be 20ms
  // if (dt < DT_POS_UPDATE*1000) return;  //dt minimum limit to 150 millis
  // lastPosUpdateMillis= curTime;
#ifdef SIMULATION

  float distanceRight = desMotSpdL * 12.0;  //(encoderPosLeft-encRighto)*0.6135923151;
  float distanceLeft = desMotSpdR  * 12.0;    //(encPosLeft-encLefto)*0.6135923151;
  liftLevel+=desLiftSpeed*DT_CONTROL/360.0*1400.0;
#else
  float liftLevelDistance = liftLevel - liftLevelo;
  float distanceRight = -(encPosRight - encRighto) * 0.69;
  float distanceLeft = -(encPosLeft - encLefto) * 0.69;
#endif
  // Serial.println(curSpeedLift);
  float distance = (distanceLeft + distanceRight) / 2.0;
  curSpeed = distance / 1000.0 / DT_POS_UPDATE;
  curSpeedL = distanceLeft / 1000.0 / DT_POS_UPDATE;
  curSpeedR = distanceRight / 1000.0 / DT_POS_UPDATE;
  curSpeedLift = liftLevelDistance/LIFT_PPR/DT_POS_UPDATE;//round per Sec
  encRighto = encPosRight;
  encLefto = encPosLeft;
  liftLevelo = liftLevel;
  liftLevelAngle=-(liftLevel-liftLevelDown)*360.0/LIFT_PPR;
  // while (liftLevelAngle<-180)liftLevelAngle+=360;
  // while (liftLevelAngle>180)liftLevelAngle-=360;
  liftAngle=botangle+liftLevelAngle;
  while (liftAngle<-180)liftAngle+=360;
  while (liftAngle>180)liftAngle-=360;
  float diff = (distanceRight- distanceLeft);
  botRotationSpeed = DEG_RAD*((diff / ( 1000.0 * BASE_LEN)))/DT_POS_UPDATE;
  // botangle += botRotationSpeed*DT_CONTROL;
  // while (botangle >= 180) botangle -= 360;
  // while (botangle < -180) botangle += 360;
  botx += sin((botangle)/DEG_RAD) * distance;
  boty += cos((botangle)/DEG_RAD) * distance;
  float angleIMU = imu_data.gyroyaw;
  if (angleIMU > 180) angleIMU -= 360;
  if (angleIMU < -180) angleIMU += 360;
  botangle = angleIMU;
  // liftLevel += liftLevel;
}
unsigned long lastDebugTime=0;

void RobotDriver::DebugReport()
{
  // Serial.println(curTime-lastDebugTime);
  if((curTime-lastDebugTime)<20)return;
  
  lastDebugTime = curTime;
  // debugCounter=0;
  debugCounter++;
  if(debugCounter>=3)debugCounter=0;    
  
  DPRINT("!$ curSpeed L R Li Ro:");  DPRINTLN(curSpeedL);  DPRINTLN(curSpeedR);  DPRINTLN(curSpeedLift); DPRINTLN(botRotationSpeed); DPRINT("#");
    DPRINT("!$DesMotSpd RLL:");  DPRINTLN(desMotSpdR);   DPRINTLN(desMotSpdL); DPRINTLN(desLiftSpeed);  DPRINT("#");
  DPRINT("!$RotSpd des cur:");     DPRINTLN(desRotSpd); DPRINTLN(imu_data.gyroZ);  DPRINT("#");
  if(debugCounter==0){
    DPRINT("!$PID yaw:");       DPRINTLN(Kp_yaw * error_yaw);DPRINTLN(Ki_yaw * integral_yaw);DPRINTLN(Kd_yaw * derivative_yaw);DPRINT("#");
    DPRINT("!$PID pos:");       DPRINTLN(Kp_pos * error_pos);DPRINTLN(Ki_pos * integral_pos);DPRINTLN(Kd_pos * derivative_pos);DPRINT("#");

  }
  if(debugCounter==1){
    DPRINT("!$dx,dy,dd,da:");  DPRINTLN(desX);  DPRINTLN(desY);  DPRINTLN(desDistance);  DPRINTLN(desAngle);  DPRINT("#");
    DPRINT("!$x,y,ba,la:");  DPRINTLN(botx);  DPRINTLN(boty);  DPRINTLN(botangle); DPRINTLN(liftAngle);  DPRINT("#");
    DPRINT("!$floorMap:");  DPRINTLN(floorMap.size());   DPRINT("#");
    
  }
  if(debugCounter==2){
  DPRINT("!$GyroYaw:");  DPRINTLN(imu_data.gyroyaw);  DPRINT("#");
  DPRINT("!$PIDerr YP:");     DPRINTLN(error_yaw);  DPRINTLN(error_pos); DPRINT("#");  
  DPRINT("!$curtime cursyns:");     DPRINTLN(curTime);  DPRINTLN(syncLossCount); DPRINT("#");  
  
//  sendSyncPacket() ;
  }
  S_DEBUG.print('@');
  S_DEBUG.flush();

}

void RobotDriver::controlLoop()
{
  int dt = curTime - lastLoopMillis;  //check dt, should be 20ms
  if (dt < 20) return;  //dt minimum limit to 20 millis
  lastLoopMillis = curTime;
  int times500ms = curTime/1000;
  syncLossCount++;
  if(syncLossCount>100)gotoMode(MODE_STANDBY);
  if(times500ms!=lastSyncSec)
  {
    sendSyncPacket();
    // reportPPU();
    lastSyncSec = times500ms;
  }
  posUpdate();
  DebugReport();
  switch (bot_mode)
  {
  case MODE_STANDBY:
    loopStandby();
    break;
  case MODE_MOVE:
    loopMove();
    break;
  case MODE_ROTATE:
    loopRotate();
    // Serial.println("loop");
    break;
  case MODE_LIFT:
    loopLift();
    break;
  default:
    break;
  }
  
}
void RobotDriver::loopMove() {

  //error calculation
  float dx = desX - botx;
  float dy = desY - boty;

  desDistance = sqrt(dx * dx + dy * dy);
  desBearing = ConvXYtoAngle(dx, dy);

  yaw_PID =  calcPIDyaw(desBearing);
  
  //PID speed
  error_pos = desDistance * cos((error_yaw)/DEG_RAD);
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
    if(stillCount>100)  gotoMode(MODE_LIFT);
    integral_pos=0;
    error_pos_prev=0;
    error_pos=0;
  }
  else
    {
    if(stillCount>0)stillCount--;
    }
  pos_PID =  calcPIDPos(error_pos);
  float desSpeed = constrainVal(pos_PID, -maxBotSpeed, maxBotSpeed);
  float maxRorationSpeed = (maxBotSpeed-abs(desSpeed))/maxBotSpeed;//high desSpeed less rotation speed
  if(desDistance<150)maxRorationSpeed*=(desDistance/150.0);//less distance less rotation
  if(maxRorationSpeed<0.05)maxRorationSpeed=0.05;
  desRotSpd =  constrainVal(yaw_PID, -maxBotRotSpd, maxBotRotSpd);
  desRotSpd*= maxRorationSpeed;  
  setSpeedRight(desSpeed- desRotSpd * BASE_LEN / 2.0);
  setSpeedLeft(desSpeed+desRotSpd * BASE_LEN / 2.0);  
  liftStabilize();

}
void RobotDriver::update() {
  if (!initOK) return;
   curTime = millis();
     
  // readSensBus
  while (portSenBus->available()) {
    unsigned char inputByte = portSenBus->read();
    if(sbus.Input(inputByte))
    {
      // imu.resetYaw(sbus.tagAngle);
    }
  }
  imu.updateData();
  imu_data = imu.getMeasurement();
  
  if (imu.getIsConnected() == false) this->isActive = false;
  //read Motor report
  while (portMotor->available()) {
    unsigned char inputByte = portMotor->read();
    processMotorReport(inputByte);
  }
  //read imcoming command
  updateCommandBus();
  controlLoop();
  
}
 
void RobotDriver::gotoMode(int mode)
{
  bot_mode = mode;
  integral_pos=0;
  error_pos_prev=0;
  error_pos=0;
  error_yaw_prev=0;
  integral_yaw=0;
  error_yaw=0;
  stillCount=0;
}
float RobotDriver::calcPIDPos(float epos)
{
  error_pos = epos;
  integral_pos += error_pos * DT_CONTROL;
  integral_pos = constrainVal(integral_pos, -i_limit_pos, i_limit_pos);  //saturate integrator to prevent unsafe buildup
  if(error_pos_prev==0)error_pos_prev=error_pos;
  derivative_pos = (error_pos - error_pos_prev) / DT_CONTROL;
  float pid_out = (Kp_pos * error_pos + Ki_pos * integral_pos + Kd_pos * derivative_pos)/1000.0;  //scaled by .01 to bring within -1 to 1 range
  error_pos_prev = error_pos;
  return pid_out;
}

float RobotDriver::calcPIDyaw(float targetAngle)
{
  error_yaw = targetAngle - botangle;
  // Serial.println(error_yaw);
  while (error_yaw > 180) error_yaw -= 360;
  while (error_yaw < -180) error_yaw += 360;
  float scaled_error_yaw=constrain(error_yaw, -60, 60);
  integral_yaw += scaled_error_yaw *DT_CONTROL*5;
  float scaledIlimit = i_limit_yaw;
  if(scaledIlimit>5*abs(scaled_error_yaw))scaledIlimit=5*abs(scaled_error_yaw);
  integral_yaw = constrainVal(integral_yaw, -scaledIlimit, scaledIlimit);  //saturate integrator to prevent unsafe buildup
  if(error_yaw_prev==0)error_yaw_prev=scaled_error_yaw;
  derivative_yaw = (scaled_error_yaw - error_yaw_prev) / DT_CONTROL;
  float pid_out = 0.1*(Kp_yaw * scaled_error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw);  
  error_yaw_prev = scaled_error_yaw;
  return pid_out;
}
void RobotDriver::setSpeedRight(float value)
{
  value = constrainVal(value, -maxBotSpeed, maxBotSpeed);
  float acc = value- desMotSpdR;//todo: use curSpeedR for better accuracy
  if(acc*desMotSpdR<0)acc = constrainVal(acc, -maxBotAcc*3, maxBotAcc*3 );
  else acc = constrainVal(acc, -maxBotAcc , maxBotAcc );
  desMotSpdR += acc;
  sendControlPacket(1, desMotSpdR/MAX_MOTION_SPEED, 0);
}
void RobotDriver::setSpeedLeft(float value)
{
  value = constrainVal(value, -maxBotSpeed, maxBotSpeed);
  float acc = value- desMotSpdL;//todo: use curSpeedL for better accuracy
  if(acc*desMotSpdL<0)acc = constrainVal(acc, -maxBotAcc*3, maxBotAcc*3);
  else acc = constrainVal(acc, -maxBotAcc , maxBotAcc );
  desMotSpdL += acc;
  sendControlPacket(2, desMotSpdL/MAX_MOTION_SPEED, 0);
}
void RobotDriver::loopRotate()
{
  yaw_PID = calcPIDyaw(desAngle);
  if (abs(error_yaw) < 2)
  {
    stillCount+=1;
    if(stillCount>100)  gotoMode(MODE_STANDBY);
    integral_pos=0;
    error_pos_prev=0;
    error_pos=0;
  }
  else if (abs(error_yaw) < 1.2)
  {
    stillCount+=2;
    if(stillCount>100)  gotoMode(MODE_STANDBY);
    integral_pos=0;
    error_pos_prev=0;
    error_pos=0;
  }
  else {
    if(stillCount>0)stillCount--;
  }

  desRotSpd = yaw_PID;  
  desRotSpd = constrainVal(desRotSpd, -maxBotRotSpd, maxBotRotSpd);
  setSpeedRight(- desRotSpd * BASE_LEN / 2.0);
  setSpeedLeft(desRotSpd * BASE_LEN / 2.0);  
  liftStabilize();
  
}
void RobotDriver::liftStabilize()
{
    desLiftSpeed = desRotSpd/8.5;//+botRotationSpeed/40.0;//+= dliftSpeed;
  desLiftSpeed = constrainVal(desLiftSpeed, -0.18, 0.18);
    if (liftLevelMinDefined && liftLevelInitOK)
   sendControlPacket(3, desLiftSpeed, 0);
}
void RobotDriver::loopStandby()
{
  // posUpdate();
  setSpeedLeft(desMotSpdL*0.9);
  setSpeedRight(desMotSpdR*0.9);
  if (liftLevelMinDefined == false)
    desMotorSpeedLift=-0.1;
    else if (liftLevelInitOK == false)
    {
      desMotorSpeedLift=(liftLevelDown-liftLevel)/1000.0;
      desMotorSpeedLift=constrainVal(desMotorSpeedLift,-0.2,0.2);
    }
  else  desMotorSpeedLift*=0.8;
  if (liftLevelMinDefined && (liftLevel>liftLevelDown)) liftLevelInitOK = true;
  sendControlPacket(3, desMotorSpeedLift, 0);
}
void RobotDriver::sendSyncPacket() {
  for(unsigned int i =0;i<paramTable.size();i++)
  {
    DPRINTF("!$PARAM:"); 
    DPRINTLN(paramTable[i].paramName);
    DPRINTLN(paramTable[i].paraValue);
    DPRINT("#");
  }
  DPRINT('@');
  DPRINT("!$lastSyncSec:");  DPRINTLN(lastSyncSec);  DPRINT("#");
  DPRINT("!$paramTable.size:");  DPRINTLN(paramTable.size());  DPRINT("#");
  DPRINT("!$GyroConnect:");  DPRINTLN(imu.getIsConnected());  DPRINT("#");
  DPRINT("!$Lift Status:");   DPRINTLN(liftLevel);  DPRINTLN(liftLevelAngle); DPRINTLN(isLiftMinPos); DPRINTLN(isLiftMaxPos);  DPRINT("#");
  DPRINT("!$Motor Fail:");    DPRINTLN(M1Fail);   DPRINTLN(M2Fail);   DPRINTLN(M3Fail);   DPRINT("#");
  DPRINT("!$bot_mode:");  DPRINTLN(bot_mode); DPRINTLN(stillCount); DPRINT("#");
  DPRINT('@');
  // uint globalmsec = curTime;
  // controlPacket[2] = 0x00;
  // controlPacket[3] = (0xff) & (globalmsec >> 16);
  // controlPacket[4] = (0xff) & (globalmsec >> 8);
  // controlPacket[5] = (0xff) & globalmsec;
  // controlPacket[6] = calcCS8(controlPacket, 6);
  // portMotor->write(controlPacket, 7);
  
}
void RobotDriver::loadParams()
{
  Kp_yaw = loadParam("Kp_yaw",0.6) ;   //Yaw P-gain
  Ki_yaw = loadParam("Ki_yaw",0.2);   //Yaw I-gain
  Kd_yaw = loadParam("Kd_yaw",0.14);  //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
  Kp_pos = loadParam("Kp_pos",1.5);  //Yaw P-gain
  Ki_pos = loadParam("Ki_pos",0.2);  //Yaw I-gain
  Kd_pos = loadParam("Kd_pos",0.1);  //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
  maxBotSpeed = loadParam("maxBotSpeed",0.4);
  maxBotRotSpd = loadParam("maxBotRotSpd",2);
  maxBotAcc = loadParam ("maxBotAcc",5.0)/1000.0;
}
void RobotDriver::sendControlPacket(uint8_t id, float speed, uint8_t mode) {
  // control left motor
  controlPacket[2] = id;
  if (id == 1) M1Fail++;
  if (id == 2) M2Fail++;
  if (id == 3) M3Fail++;
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
uint8_t reportPacket[50];
int MotorReportBuffID = 0;
void RobotDriver::processMotorReport(uint8_t bytein) {
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
          M3Fail = 0;
          uint8_t cs = calcCS8(reportPacket, 7);
          if (cs == reportPacket[7]) {
            isLiftMinPos = (reportPacket[5] == 0);
            isLiftMaxPos = (reportPacket[6] == 0);
            if (isLiftMinPos&&(liftLevelMinDefined==false)) 
            {
              liftLevelMinDefined = true;
              // desLiftLevel=liftLevelDown;
              // gotoMode(MODE_LIFT);
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
          M2Fail = 0;
        }
        if (reportPacket[2] == 0x81)  //lift motor report
        {
          M1Fail = 0;
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
void RobotDriver::loopLift()
{
  switch (liftComm)
  {
  case 0:
    desLiftLevel = liftLevelDown;
    break;
  case 1:
    desLiftLevel = liftLevelUp;
    break;
  case 2:
    desLiftLevel = liftLevelDown;
    break;
  case 3:
    desLiftLevel = liftLevelUp;
    break;
  default:
    break;
  }
  float desLiftLevelError = desLiftLevel-liftLevel;
  if(abs(desLiftLevelError)<1000)stillCount++;
  else stillCount=0;
  if(abs(desLiftLevelError)<1000&&stillCount>0)gotoMode(MODE_ROTATE);
  desLiftSpeed = desLiftLevelError/1100.0;
  desLiftSpeed = constrainVal(desLiftSpeed, -0.1, 0.1);
  sendControlPacket(3, desLiftSpeed, 0);
  
  // curSpeed
  desRotSpd = desLiftSpeed*9.4;//botRotationSpeed*9.2;
  // desRotSpd=desRotSpd-liftAngle/30.0;
  desRotSpd = constrainVal(desRotSpd, -1.3, 1.3);
  float acc = 0 - desRotSpd * BASE_LEN / 2.0 - desMotSpdR;
  acc = constrainVal(acc, -maxBotAcc , maxBotAcc );
  desMotSpdR += acc;
  desMotSpdR = constrainVal(desMotSpdR, -1.0, 1.0);
  acc = 0 + desRotSpd * BASE_LEN / 2.0 - desMotSpdL;
  acc = constrainVal(acc, -maxBotAcc , maxBotAcc );
  desMotSpdL += acc;
  // desMotSpdL*=1.08;
  desMotSpdL = constrainVal(desMotSpdL, -1.0, 1.0);
  
  // float errorLift = desLiftLevel-liftLevel;
  // float newdesLiftSpeed = botRotationSpeed/4.0;
  // float dliftSpeed = newdesLiftSpeed-desLiftSpeed;
  // dliftSpeed = constrainVal(dliftSpeed,-0.02/DT_CONTROL,0.02/DT_CONTROL);
  // desLiftSpeed = botRotationSpeed/DEG_RAD/7.8;//+= dliftSpeed;
  sendControlPacket(1, desMotSpdR, 0);
  sendControlPacket(2, desMotSpdL, 0);
  // if (liftLevelMinDefined && liftLevelMaxDefined)
  //  sendControlPacket(3, desLiftSpeed, 0);
}