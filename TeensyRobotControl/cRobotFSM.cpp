// #include "core_pins.h"
#include <stdint.h>

// #include "usb_serial.h"
#include <math.h>

#include "cRobotFSM.h"
long int encoderPosLeft = 0;
long int encoderPosRight = 0;
long int encoderPosLefto = 0;
long int encoderPosRighto = 0;
long int liftLevel = 0;
// long int reportCount = 0;
bool liftLevelMinDefined = false;
bool liftLevelMaxDefined = false;
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
    if(liftLevel>2000)liftLevelMaxDefined = true;
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

      if (tokens[1].equals("m")) {


        float comX = (tokens[2].toFloat());
        float comY = tokens[3].toFloat();
        desX = comX;
        desY = comY;
        motionMode = 1;
        // DPRINTLN(comX);
        // DPRINTLN(comY);
      }
      if (tokens[1].equals("a")) {

        desAngle = (tokens[2].toFloat());
        // float comY = tokens[3].toFloat();
        // desX = comX;
        // desY = comY;
        motionMode = 1;
        // DPRINTLN(comX);
        // DPRINTLN(comY);
      }
      if (tokens[1].equals("d")) {
        
        float distance = tokens[2].toFloat()*1000;
        // desAngle = (tokens[2].toFloat());
        // float comY = tokens[3].toFloat();
        desX = desX+distance*sin(desAngle/DEG_RAD);
        desY = desY+distance*cos(desAngle/DEG_RAD);
        motionMode = 1;
        // Serial.print("$!COM,");
        // Serial.println(desX);
        // Serial.println(desY);
        // Serial.print('#');
        // DPRINTLN(comX);
        // DPRINTLN(comY);
      }
      if (tokens[1].equals("r")) {

        desAngle += (tokens[2].toFloat())*90;
        // float comY = tokens[3].toFloat();
        // desX = desX+sin(radians(desAngle));
        // desY = desY+cos(radians(desAngle));
        motionMode = 1;
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
    // if((bytein==0xab)&&(bytein1==0x55)&&(bytein2==0xa0))
    // {
    //   debugCounter=0 ;
    //     // DebugReport();
    // }
    // if((bytein==0xab)&&(bytein1==0x55)&&(bytein2==0xa1))
    // {
    //   debugCounter=1;
    //     // DebugReport();
    // }
    // if((bytein==0xab)&&(bytein1==0x55)&&(bytein2==0xa2))
    // {
    //   debugCounter=2;
    //     // DebugReport();
    // }
    // if((bytein==0xab)&&(bytein1==0x55)&&(bytein2==0xa2))
    // {
    //   debugCounter=3;
    //     // DebugReport();
    // }
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
  S_DEBUG.begin(921600);    //IMU
  S_SENSORS.begin(1000000);  //sens bus
  S_MOTORS.begin(230400);
  S_COMMAND.begin(57600);
  S_DEBUG.begin(2000000);
  portIMU = &S_DEBUG;
  portSenBus = &S_SENSORS;
  portMotor = &S_MOTORS;
  imu.IMU_init(portIMU);
  Serial.println("start");
#ifdef SIMULATION
  liftLevel=0;
  liftLevelMaxDefined=true;
  liftLevelMinDefined = true;
  desLiftLevel = 7000;
#endif
  if (imu.getIsConnected()) {
    Serial.println("IMU connect OK");

  } else {
    Serial.println("IMU connect failed");
    blink(3);
  }
  DPRINTF("RobotDriver Setup");
  Serial.flush();

  encoderPos = 0;
  botRotationSpeed = 10.0;
  i_limit_pos = 50;
  pinMode(EMG_STOP, INPUT);
  pinMode(ENC_A1, INPUT);
  pinMode(ENC_B1, INPUT);
  pinMode(ENC_A2, INPUT);
  pinMode(ENC_B2, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A1), EncIntLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), EncIntRight, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A3), EncIntLift, RISING);
  Kp_yaw = 0.4;   //Yaw P-gain
  Ki_yaw = 0.2;   //Yaw I-gain
  Kd_yaw = 0.01;  //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
  desAngle = 0;
  Kp_pos = 0.010;  //Yaw P-gain
  Ki_pos = 0.006;  //Yaw I-gain
  Kd_pos = 0.002;  //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
  controlPacket[0] = 0xaa;
  controlPacket[1] = 0x55;
  controlPacket[2] = 0x00;
  initOK = true;

}
void RobotDriver::gotoStandby() {
}
void RobotDriver::posUpdate() {
#ifdef SIMULATION

  float distanceRight = desMotSpdL * 12.0;  //(encoderPosLeft-encoderPosLefto)*0.6135923151;
  float distanceLeft = desMotSpdR  * 12.0;    //(encoderPosRight-encoderPosRighto)*0.6135923151;
  liftLevel+=desLiftSpeed*DT_CONTROL/360.0*1400.0;
#else
  float distanceLeft = -(encoderPosLeft - encoderPosLefto) * 0.6135923151;
  float distanceRight = -(encoderPosRight - encoderPosRighto) * 0.6135923151;
#endif
  float distance = (distanceLeft + distanceRight) / 2.0;
  curSpeed = distance / 1000.0 / DT_CONTROL;
  encoderPosLefto = encoderPosLeft;
  encoderPosRighto = encoderPosRight;
  float diff = (distanceRight- distanceLeft);
  botRotationSpeed = DEG_RAD*((diff / ( 1000.0 * BASE_LEN)))/DT_CONTROL;
  botangle += botRotationSpeed*DT_CONTROL;
  while (botangle >= 180) botangle -= 360;
  while (botangle < -180) botangle += 360;
  botx += sin((botangle)/DEG_RAD) * distance;
  boty += cos((botangle)/DEG_RAD) * distance;
 
  // liftLevel += liftLevel;
}
unsigned long lastDebugTime=0;

void RobotDriver::DebugReport()
{
  // Serial.println(curTime-lastDebugTime);
  if((curTime-lastDebugTime)<50)return;
  lastDebugTime = curTime;
  // debugCounter=0;
  debugCounter++;
  if(debugCounter>3)debugCounter=0;
  if(debugCounter==0){
  DPRINT("!$dx,dy,dd,da:");  DPRINTLN(desX);  DPRINTLN(desY);  DPRINTLN(desDistance);  DPRINTLN(desBearing);  DPRINT("#");
  DPRINT("!$ x,y,ang,spd:");  DPRINTLN(botx);  DPRINTLN(boty);  DPRINTLN(botangle);  DPRINTLN(curSpeed);  DPRINT("#");
  }

  if(debugCounter==1){
  DPRINT("!$PID yaw:");       DPRINTLN(Kp_yaw * error_yaw);DPRINTLN(Ki_yaw * integral_yaw);DPRINTLN(Kd_yaw * derivative_yaw);DPRINT("#");
  DPRINT("!$PID pos:");       DPRINTLN(Kp_pos * error_pos);DPRINTLN(Ki_pos * integral_pos);DPRINTLN(Kd_pos * derivative_pos);DPRINT("#");
  DPRINT("!$error yaw pos:");     DPRINTLN(error_yaw);  DPRINTLN(error_pos); DPRINT("#");  
  }
  if(debugCounter==2){
  DPRINT("!$Lift Status:");   DPRINTLN(liftLevel);   DPRINTLN(liftLevel / 1400.0);   DPRINT("#");
  DPRINT("!$Motor Fail:");    DPRINTLN(M1Fail);   DPRINTLN(M2Fail);   DPRINTLN(M3Fail);   DPRINT("#");
  }
  if(debugCounter==3){
  DPRINT("!$DesMotSpd RLL:");  DPRINTLN(desMotSpdR);   DPRINTLN(desMotSpdL); DPRINTLN(desLiftSpeed);  DPRINT("#");
  DPRINT("!$desRotSpd:");     DPRINTLN(desRotSpd);   DPRINT("#");
  DPRINT("!$Limits:");        DPRINTLN(isLiftMinPos); DPRINTLN(isLiftMaxPos);  DPRINT("#");
  }
  S_DEBUG.print('@');

}
void RobotDriver::calculateControlLoop() {

  int dt = curTime - timeMillis;  //check dt, should be 20ms
  if (dt < 20) return;  //dt minimum limit to 20 millis
  sendSyncPacket();
  timeMillis = curTime;
  posUpdate();
  
  // portIMU->println("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");

  //error calculation
  float dx = desX - botx;
  float dy = desY - boty;

  desDistance = sqrt(dx * dx + dy * dy);
  desBearing = ConvXYtoAngle(dx, dy);

  float angleIMU = -imu_data.gyroyaw;
  if (angleIMU > 180) angleIMU -= 360;
  if (angleIMU < -180) angleIMU += 360;

  float targetAngle = desAngle;
  if (motionMode == 1) targetAngle = desBearing;
  // desBearing= 0;
  error_yaw = targetAngle - botangle;

  while (error_yaw > 180) error_yaw -= 360;
  while (error_yaw < -180) error_yaw += 360;

  integral_yaw += error_yaw * DT_CONTROL;
  integral_yaw = constrainVal(integral_yaw, -i_limit_yaw, i_limit_yaw);  //saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) / DT_CONTROL;
  yaw_PID = (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw);  //scaled by .01 to bring within -1 to 1 range
  error_yaw_prev = error_yaw;

  // calculate curSpeed
  desRotSpd = 0.1 * yaw_PID * (1.0 - curSpeed);  //high curSpeed less rotation
  desRotSpd = constrainVal(desRotSpd, -1, 1);
  //PID speed
  error_pos = 0;
  if (motionMode == 1) {
    if (abs(error_yaw < 90)) {
      error_pos = desDistance * cos((error_yaw)/DEG_RAD);
    }
    if(desDistance<50)
    {
      motionMode=0;
      integral_pos=0;
      error_pos_prev=0;
      error_pos=0;
    }
  }

  integral_pos += error_pos * DT_CONTROL;
  integral_pos = constrainVal(integral_pos, -i_limit_pos, i_limit_pos);  //saturate integrator to prevent unsafe buildup
  derivative_pos = (error_pos - error_pos_prev) / DT_CONTROL;
  pos_PID = .3 * (Kp_pos * error_pos + Ki_pos * integral_pos + Kd_pos * derivative_pos);  //scaled by .01 to bring within -1 to 1 range
  error_pos_prev = error_pos;
  float desSpeed = constrainVal(pos_PID, -0.32, 0.32);
  float acc = desSpeed - desRotSpd * BASE_LEN / 2.0 - desMotSpdR;
  acc = constrainVal(acc, -ACC_MAX * 2, ACC_MAX * 2);
  desMotSpdR += acc;
  desMotSpdR = constrainVal(desMotSpdR, -1.0, 1.0);
  acc = desSpeed + desRotSpd * BASE_LEN / 2.0 - desMotSpdL;
  acc = constrainVal(acc, -ACC_MAX * 2, ACC_MAX * 2);
  desMotSpdL += acc;
  desMotSpdL*=1.08;
  desMotSpdL = constrainVal(desMotSpdL, -1.0, 1.0);
  
  // float errorLift = desLiftLevel-liftLevel;
  // float newdesLiftSpeed = botRotationSpeed/4.0;
  // float dliftSpeed = newdesLiftSpeed-desLiftSpeed;
  // dliftSpeed = constrainVal(dliftSpeed,-0.02/DT_CONTROL,0.02/DT_CONTROL);
  desLiftSpeed =botRotationSpeed/DEG_RAD/7.8;//+= dliftSpeed;
  sendControlPacket(1, desMotSpdR, 0);
  sendControlPacket(2, desMotSpdL, 0);
  if (liftLevelMinDefined == false)
    sendControlPacket(3, -0.2, 0);
  else if (liftLevelMaxDefined == false)
    sendControlPacket(3, 0.2, 0);
  else sendControlPacket(3, desLiftSpeed, 0);
}
void RobotDriver::update() {
  if (!initOK) return;
   curTime = millis();
     
  // readSensBus
  while (portSenBus->available()) {
    unsigned char inputByte = portSenBus->read();
    sbus.Input(inputByte);
  }
  // DPRINTF("Check Setup");
  Serial.flush();
  //read IMU
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
  calculateControlLoop();
  DebugReport();
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
    desLiftSpeed = speed;
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
            if (isLiftMinPos) liftLevelMinDefined = true;
            if (liftLevelMinDefined && isLiftMaxPos) liftLevelMaxDefined = true;
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
