// #include "core_pins.h"
#include <stdint.h>
#define LIFT_PPR 1392.64//1360
// #include "usb_serial.h"
#include <math.h>

#include "cRobotFSM.h"
long int encoderPosLeft = 0;
long int encoderPosRight = 0;
long int encoderPosLefto = 0;
long int encoderPosRighto = 0;
long int liftLevel = 0;
long int liftLevelo = 0;
// long int reportCount = 0;
bool liftLevelMinDefined = false;
bool liftLevelInitOK = false;
int desLiftLevel = 0;
int M1Fail = 0, M2Fail = 0, M3Fail = 0;

#define CONTROL_LEN 10
// #define MAX_LIFT_H 12000
int liftMaxLevel = 0;
unsigned char controlPacket[CONTROL_LEN];
void EncIntLeft() {
  if (digitalRead(ENC_B1) > 0)
    encoderPosLeft++;
  else encoderPosLeft--;
}
void EncIntRight() {
  if (digitalRead(ENC_B2) > 0)
    encoderPosRight--;
  else encoderPosRight++;
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
void RobotDriver::processCommand(String command) {

  std::vector<String> tokens = splitString(command,',');
  // Serial.print("!$command:");
  //         Serial.println(tokens.size());
  //         Serial.print("#");
  if (tokens.size() >= 2) {
    if (tokens[0].indexOf("$COM")>=0) {
      DPRINT("!$Command:");  DPRINTLN(command);  DPRINT("#");
      if (tokens[1].equals("m")) {


        float comX = (tokens[2].toFloat());
        float comY = tokens[3].toFloat();
        desX = comX;
        desY = comY;
        gotoMode(MODE_MOVE);
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
        imu.resetYaw();
        // DPRINTLN(comX);
        // DPRINTLN(comY);
      }
      if (tokens[1].equals("lift")) {

        desLiftLevel += (tokens[2].toFloat())*LIFT_PPR;
        gotoMode(MODE_LIFT);
        DPRINT("!$Command:");  DPRINTLN(command);  DPRINT("#");
        
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
  while (S_COMMAND.available()) {
    
    uint8_t bytein = S_COMMAND.read();
    if((bytein==0xaa)&&(bytein1==0x55)&&(bytein2==0xaa))
    {
        commandMessageI=0;
    }
    
    // Serial.println((bytein));
    bytein2 = bytein1;
    bytein1 = bytein;
    // if(commandMessageI>=49)commandMessageI=-1;
    // if(commandMessageI>=0)
    // {
    //   commandMessage[commandMessageI]=bytein;
    //   if(commandMessageI==1)if(bytein!=0xF2)commandMessageI=-1;//check address
    //   if(commandMessageI==4)
    //   {
    //     commandCode = commandMessage[3]<<8+commandMessage[4];
    //   }
    //   commandMessageI++;
    // }
    if(isPrintable (bytein))
    { 
      
      if (commandString.length() < COMMAND_LEN_MAX) commandString += (char)bytein;
      else commandString = "";
      if (bytein == '\n')  //end of command
      {
        // int dataLen = commandString.length();
        // S_DEBUG.println(dataLen);
        
        
        if (commandString.startsWith("yaw="))  //angle set command
        {

          // float angle
          //   S_DEBUG.print(imu.gyroZBiasCompensation*100000);
          // yaw_des = commandString.substring(4, dataLen - 1).toFloat();
          // S_DEBUG.print("yaw_des set:");
          // S_DEBUG.println(yaw_des);
          //   S_DEBUG.println(yaw_IMU);
        } else if (commandString.startsWith("pos="))  //angle set command
        {

          // float angle
          //   S_DEBUG.print(imu.gyroZBiasCompensation*100000);
          // pos_des = commandString.substring(4, dataLen - 1).toFloat();
          // S_DEBUG.print("pos_des set:");
          // S_DEBUG.println(pos_des);
          //   S_DEBUG.println(yaw_IMU);
        } else if (commandString.startsWith("resetyaw"))  //angle set command
        {
          // imu.resetYaw();
        } else if (commandString.startsWith("stt="))  //active set command
        {

          // float angle
          //   S_DEBUG.print(imu.gyroZBiasCompensation*100000);
          // int newstat = commandString.substring(4, dataLen - 1).toFloat();
          // gotoState(newstat);
          //   S_DEBUG.println(yaw_IMU);
        } else if (commandString.indexOf("$COM")>=0) {
          processCommand(commandString);
          
        }
        commandString = "";
      }
    }
  
  }
}
RobotDriver::RobotDriver() {
  S_IMU.begin(921600);    //IMU
  S_SENSORS.begin(1000000);  //sens bus
  S_MOTORS.begin(230400);
  S_COMMAND.begin(57600);
  S_DEBUG.begin(2000000);
  portIMU = &S_IMU;
  portSenBus = &S_SENSORS;
  portMotor = &S_MOTORS;
  imu.IMU_init(portIMU);
  Serial.println("start");
  desLiftLevel = liftLevelA0;
#ifdef SIMULATION
  liftLevel=0;
  liftLevelA0 = LIFT_PPR/4.0;
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

  attachInterrupt(digitalPinToInterrupt(ENC_A1), EncIntLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), EncIntRight, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A3), EncIntLift, RISING);
  Kp_yaw = 0.8;   //Yaw P-gain
  Ki_yaw = 0.2;   //Yaw I-gain
  Kd_yaw = 0.03;  //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
  desAngle = 0;
  Kp_pos = 0.010;  //Yaw P-gain
  Ki_pos = 0.002;  //Yaw I-gain
  Kd_pos = 0.001;  //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
  controlPacket[0] = 0xaa;
  controlPacket[1] = 0x55;
  controlPacket[2] = 0x00;
  initOK = true;

}
void RobotDriver::gotoStandby() {
}
void RobotDriver::posUpdate() {
  int dt = curTime - lastPosUpdateMillis;  //check dt, should be 20ms
  if (dt < 50) return;  //dt minimum limit to 150 millis
  lastPosUpdateMillis= curTime;
#ifdef SIMULATION

  float distanceRight = desMotSpdL * 12.0;  //(encoderPosLeft-encoderPosLefto)*0.6135923151;
  float distanceLeft = desMotSpdR  * 12.0;    //(encoderPosRight-encoderPosRighto)*0.6135923151;
  liftLevel+=desLiftSpeed*DT_CONTROL/360.0*1400.0;
#else
  float liftLevelDistance = liftLevel - liftLevelo;
  float distanceLeft = -(encoderPosLeft - encoderPosLefto) * 0.69;
  float distanceRight = -(encoderPosRight - encoderPosRighto) * 0.69;
#endif
  // Serial.println(curSpeedLift);
  float distance = (distanceLeft + distanceRight) / 2.0;
  curSpeed = distance / 1000.0 / DT_CONTROL;
  curSpeedL = distanceLeft / 1000.0 / DT_CONTROL;
  curSpeedR = distanceRight / 1000.0 / DT_CONTROL;
  curSpeedLift = liftLevelDistance/LIFT_PPR/DT_CONTROL;//round per Sec
  encoderPosLefto = encoderPosLeft;
  encoderPosRighto = encoderPosRight;
  liftLevelo = liftLevel;
  liftLevelAngle=-(liftLevel-liftLevelA0)*360.0/LIFT_PPR;
  // while (liftLevelAngle<-180)liftLevelAngle+=360;
  // while (liftLevelAngle>180)liftLevelAngle-=360;
  liftAngle=botangle+liftLevelAngle;
  while (liftAngle<-180)liftAngle+=360;
  while (liftAngle>180)liftAngle-=360;
  float diff = (distanceRight- distanceLeft);
  botRotationSpeed = DEG_RAD*((diff / ( 1000.0 * BASE_LEN)))/DT_CONTROL;
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
  if(debugCounter>3)debugCounter=0;    
  DPRINT("!$ x,y,ba,la:");  DPRINTLN(botx);  DPRINTLN(boty);  DPRINTLN(botangle); DPRINTLN(liftAngle);  DPRINT("#");
  DPRINT("!$ curSpeed L R Li Ro:");  DPRINTLN(curSpeedL);  DPRINTLN(curSpeedR);  DPRINTLN(curSpeedLift); DPRINTLN(botRotationSpeed); DPRINT("#");
  if(debugCounter==0){
    DPRINT("!$PID yaw:");       DPRINTLN(Kp_yaw * error_yaw);DPRINTLN(Ki_yaw * integral_yaw);DPRINTLN(Kd_yaw * derivative_yaw);DPRINT("#");
    DPRINT("!$PID pos:");       DPRINTLN(Kp_pos * error_pos);DPRINTLN(Ki_pos * integral_pos);DPRINTLN(Kd_pos * derivative_pos);DPRINT("#");
    DPRINT("!$error yaw pos:");     DPRINTLN(error_yaw);  DPRINTLN(error_pos); DPRINT("#");  
  }
  if(debugCounter==1){
    DPRINT("!$dx,dy,dd,da:");  DPRINTLN(desX);  DPRINTLN(desY);  DPRINTLN(desDistance);  DPRINTLN(desAngle);  DPRINT("#");
    DPRINT("!$Lift Status:");   DPRINTLN(liftLevel);  DPRINTLN(liftLevelAngle); DPRINTLN(isLiftMinPos); DPRINTLN(isLiftMaxPos);  DPRINT("#");
    DPRINT("!$Motor Fail:");    DPRINTLN(M1Fail);   DPRINTLN(M2Fail);   DPRINTLN(M3Fail);   DPRINT("#");
  }
  if(debugCounter==2){
  DPRINT("!$DesMotSpd RLL:");  DPRINTLN(desMotSpdR);   DPRINTLN(desMotSpdL); DPRINTLN(desLiftSpeed);  DPRINT("#");
  DPRINT("!$desRotSpd:");     DPRINTLN(desRotSpd);   DPRINT("#");
  }
  if(debugCounter==3){
  DPRINT("!$GyroYaw:");  DPRINTLN(imu_data.gyroyaw);  DPRINT("#");
  DPRINT("!$GyroConnect:");  DPRINTLN(imu.getIsConnected());  DPRINT("#");
  DPRINT("!$bot_mode:");  DPRINTLN(bot_mode); DPRINTLN(stillCount); DPRINT("#");
  }
  S_DEBUG.print('@');

}
void RobotDriver::controlLoop()
{
  int dt = curTime - lastLoopMillis;  //check dt, should be 20ms
  if (dt < 20) return;  //dt minimum limit to 20 millis
  lastLoopMillis = curTime;
  sendSyncPacket();
  posUpdate();
  
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

  
  float targetAngle = desBearing;
  // desBearing= 0;
  error_yaw = targetAngle - botangle;
  if(desDistance<100)error_yaw=0;
  while (error_yaw > 180) error_yaw -= 360;
  while (error_yaw < -180) error_yaw += 360;

  integral_yaw += error_yaw * DT_CONTROL*5;
  integral_yaw = constrainVal(integral_yaw, -i_limit_yaw, i_limit_yaw);  //saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) / DT_CONTROL;
  yaw_PID = (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw);  //scaled by .01 to bring within -1 to 1 range
  error_yaw_prev = error_yaw;  
  //PID speed
  error_pos = desDistance * cos((error_yaw)/DEG_RAD);
  
    if (abs(error_yaw) > 10) {
      error_pos/=(abs(error_yaw)/10.0);
    }

    if((abs(error_pos) < 30)&&(abs(desMotSpdL)<0.1)&&(abs(desMotSpdR)<0.1))
    {
      stillCount++;
      if(stillCount>100)  gotoMode(MODE_ROTATE);
      integral_pos=0;
      error_pos_prev=0;
      error_pos=0;
    }
    else
     {
      if(stillCount>0)stillCount--;
     }
  

  integral_pos += error_pos * DT_CONTROL;
  integral_pos = constrainVal(integral_pos, -i_limit_pos, i_limit_pos);  //saturate integrator to prevent unsafe buildup
  derivative_pos = (error_pos - error_pos_prev) / DT_CONTROL;
  pos_PID = .12 * (Kp_pos * error_pos + Ki_pos * integral_pos + Kd_pos * derivative_pos);  //scaled by .01 to bring within -1 to 1 range
  error_pos_prev = error_pos;
  float desSpeed = constrainVal(pos_PID, -0.32, 0.32);
  // calculate curSpeed
  // if(desSpeed)
  desRotSpd = 0.1 * yaw_PID * (1.0 - abs(desSpeed));  //high curSpeed less rotation
  desRotSpd = constrainVal(desRotSpd, -1, 1);
  float acc = desSpeed - desRotSpd * BASE_LEN / 2.0 - desMotSpdR;
  acc = constrainVal(acc, -ACC_MAX , ACC_MAX );
  desMotSpdR += acc;
  desMotSpdR = constrainVal(desMotSpdR, -1.0, 1.0);
  acc = desSpeed + desRotSpd * BASE_LEN / 2.0 - desMotSpdL;
  acc = constrainVal(acc, -ACC_MAX , ACC_MAX );
  desMotSpdL += acc;
  desMotSpdL*=1.08;
  desMotSpdL = constrainVal(desMotSpdL, -1.0, 1.0);
  
  // float errorLift = desLiftLevel-liftLevel;
  // float newdesLiftSpeed = botRotationSpeed/4.0;
  // float dliftSpeed = newdesLiftSpeed-desLiftSpeed;
  // dliftSpeed = constrainVal(dliftSpeed,-0.02/DT_CONTROL,0.02/DT_CONTROL);
liftStabilize();
  sendControlPacket(1, desMotSpdR, 0);
  sendControlPacket(2, desMotSpdL, 0);

}
void RobotDriver::update() {
  if (!initOK) return;
   curTime = millis();
     
  // readSensBus
  while (portSenBus->available()) {
    unsigned char inputByte = portSenBus->read();
    sbus.Input(inputByte);
  }
  imu.updateData();
  if (imu.isUpdated) {
    imu_data = imu.getMeasurement();
  }
  if (imu.getIsConnected() == false) this->isActive = false;
  //read Motor report
  while (portMotor->available()) {
    unsigned char inputByte = portMotor->read();
    processMotorReport(inputByte);
  }
  //read imcoming command
  updateCommandBus();
  controlLoop();
  DebugReport();
}
 
void RobotDriver::gotoMode(int mode)
{
  bot_mode = mode;
}
void RobotDriver::loopRotate()
{
  float targetAngle = desAngle;
  error_yaw = targetAngle - botangle;
  // Serial.println(error_yaw);
  while (error_yaw > 180) error_yaw -= 360;
  while (error_yaw < -180) error_yaw += 360;
  integral_yaw += error_yaw * DT_CONTROL*5;
  integral_yaw = constrainVal(integral_yaw, -i_limit_yaw, i_limit_yaw);  //saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) / DT_CONTROL;
  yaw_PID = (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw);  //scaled by .01 to bring within -1 to 1 range
  error_yaw_prev = error_yaw;
  if (abs(error_yaw) < 0.8)
  {
    stillCount++;
    if(stillCount>100)  gotoMode(MODE_STANDBY);
    integral_pos=0;
    error_pos_prev=0;
    error_pos=0;
  }
  else {if(stillCount>0)stillCount--;}
  desRotSpd = 0.1 * yaw_PID;  //high curSpeed less rotation
  desRotSpd = constrainVal(desRotSpd, -1, 1);
  float acc = 0 - desRotSpd * BASE_LEN / 2.0 - desMotSpdR;
  acc = constrainVal(acc, -ACC_MAX , ACC_MAX );
  desMotSpdR += acc;
  desMotSpdR = constrainVal(desMotSpdR, -1.0, 1.0);
  acc = 0 + desRotSpd * BASE_LEN / 2.0 - desMotSpdL;
  acc = constrainVal(acc, -ACC_MAX , ACC_MAX );
  desMotSpdL += acc;
  // desMotSpdL*=1.08;
  desMotSpdL = constrainVal(desMotSpdL, -1.0, 1.0);
  
  // float errorLift = desLiftLevel-liftLevel;
  // float newdesLiftSpeed = botRotationSpeed/4.0;
  // float dliftSpeed = newdesLiftSpeed-desLiftSpeed;
  // dliftSpeed = constrainVal(dliftSpeed,-0.02/DT_CONTROL,0.02/DT_CONTROL);
  liftStabilize();
  sendControlPacket(1, desMotSpdR, 0);
  sendControlPacket(2, desMotSpdL, 0);

  
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
  desMotSpdR*=0.9;
  desMotSpdL*=0.9;
  
  sendControlPacket(1,desMotSpdR,0);
  sendControlPacket(2,desMotSpdL,0);
  if (liftLevelMinDefined == false)
    desMotorSpeedLift=-0.1;
    else if (liftLevelInitOK == false)
    {
      desMotorSpeedLift=(liftLevelA0-liftLevel)/1000.0;
      desMotorSpeedLift=constrainVal(desMotorSpeedLift,-0.2,0.2);
    }
    
  else  desMotorSpeedLift*=0.8;
  if (liftLevelMinDefined && (liftLevel>liftLevelA0)) liftLevelInitOK = true;
  sendControlPacket(3, desMotorSpeedLift, 0);
}
void RobotDriver::sendSyncPacket() {
  uint globalmsec = curTime;
  controlPacket[2] = 0x00;
  controlPacket[3] = (0xff) & (globalmsec >> 16);
  controlPacket[4] = (0xff) & (globalmsec >> 8);
  controlPacket[5] = (0xff) & globalmsec;
  controlPacket[6] = calcCS8(controlPacket, 6);
  portMotor->write(controlPacket, 7);
}
void RobotDriver::sendControlPacket(uint8_t id, float speed, uint8_t mode) {
  // control left motor
  controlPacket[2] = id;
  if (id == 1) M1Fail++;
  if (id == 2) M2Fail++;
  if (id == 3) {
    M3Fail++;
  }
  if (speed > 0) controlPacket[4] = (0xAB);
  else controlPacket[4] = (0xBA);
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
              // desLiftLevel=liftLevelA0;
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
  float desLiftLevelError = desLiftLevel-liftLevel;
  desLiftSpeed = desLiftLevelError/1100.0;
  desLiftSpeed = constrainVal(desLiftSpeed, -0.1, 0.1);
  sendControlPacket(3, desLiftSpeed, 0);
  if(abs(desLiftLevelError)<10)stillCount++;
  else stillCount=0;
  if(abs(desLiftLevelError)<10&&stillCount>100)gotoMode(MODE_STANDBY);
  // curSpeed
  desRotSpd = desLiftSpeed*9.4;//botRotationSpeed*9.2;
  // desRotSpd=desRotSpd-liftAngle/30.0;
  desRotSpd = constrainVal(desRotSpd, -1.3, 1.3);
  float acc = 0 - desRotSpd * BASE_LEN / 2.0 - desMotSpdR;
  acc = constrainVal(acc, -ACC_MAX , ACC_MAX );
  desMotSpdR += acc;
  desMotSpdR = constrainVal(desMotSpdR, -1.0, 1.0);
  acc = 0 + desRotSpd * BASE_LEN / 2.0 - desMotSpdL;
  acc = constrainVal(acc, -ACC_MAX , ACC_MAX );
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