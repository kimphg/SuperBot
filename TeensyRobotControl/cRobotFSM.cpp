
#include "cRobotFSM.h"
long int encoderPosLeft=0;
long int encoderPosRight=0;
long int liftLevel = 0;
#define CONTROL_LEN 10
unsigned char controlPacket[CONTROL_LEN];
void EncIntLeft()
{
  if(digitalRead(ENC_B1)>0)
  encoderPosLeft++;
  else encoderPosLeft--;
  
}
void EncIntRight()
{
  if(digitalRead(ENC_B2)>0)
  encoderPosRight--;
  else encoderPosRight++;
  
}
void EncIntLift()
{
  if(digitalRead(ENC_B3)>0)
  liftLevel--;
  else liftLevel++;
  
}
float constrainVal(float input,float min, float max)
{
  if(input<min)return min;
  if(input>max)return max;
  return input;
}
void RobotDriver::processCommand(String command)
{
  DPRINTF("Motion command start:");
  std::vector<String> tokens = splitString(command);
  if(tokens.size()>=2)
  {
    if(tokens[0].equals("$COM"))
    {
      if(tokens[1].equals("m"))
      {
          
          float comX = (tokens[2].toFloat());
          float comY = tokens[3].toFloat();
          DPRINTLN(comX);
          DPRINTLN(comY);
      }
    }
  }
  else {DPRINTLNF("Command too short");}

}
String commandString;
void RobotDriver::updateCommandBus() {
  while (RS485_COM_INPUT.available()) {
    uint8_t bytein = RS485_COM_INPUT.read();
    // DEBUG_TELEMETRY.write(bytein);
    if (commandString.length() < COMMAND_LEN_MAX) commandString += (char)bytein;
    else commandString="";
    if (bytein == '\n')  //end of command
    {
      // int dataLen = commandString.length();
      // DEBUG_TELEMETRY.println(dataLen);

      if (commandString.startsWith("yaw="))  //angle set command
      {

        // float angle
        //   DEBUG_TELEMETRY.print(imu.gyroZBiasCompensation*100000);
        // yaw_des = commandString.substring(4, dataLen - 1).toFloat();
        // DEBUG_TELEMETRY.print("yaw_des set:");
        // DEBUG_TELEMETRY.println(yaw_des);
        //   DEBUG_TELEMETRY.println(yaw_IMU);
      } else if (commandString.startsWith("pos="))  //angle set command
      {

        // float angle
        //   DEBUG_TELEMETRY.print(imu.gyroZBiasCompensation*100000);
        // pos_des = commandString.substring(4, dataLen - 1).toFloat();
        // DEBUG_TELEMETRY.print("pos_des set:");
        // DEBUG_TELEMETRY.println(pos_des);
        //   DEBUG_TELEMETRY.println(yaw_IMU);
      } else if (commandString.startsWith("resetyaw"))  //angle set command
      {
        // imu.resetYaw();
      } else if (commandString.startsWith("stt="))  //active set command
      {

        // float angle
        //   DEBUG_TELEMETRY.print(imu.gyroZBiasCompensation*100000);
        // int newstat = commandString.substring(4, dataLen - 1).toFloat();
        // gotoState(newstat);
        //   DEBUG_TELEMETRY.println(yaw_IMU);
      }
      else if(commandString.startsWith("$COM"))
      {
        processCommand(commandString);
      }
      commandString = "";
    }
  }
}
RobotDriver::RobotDriver()
{
  RS485_IMU.begin(921600);//IMU
  RS485_SENS.begin(1000000);//sens bus
  RS485_MOTORS.begin(115200);
  RS485_COM_INPUT.begin(115200);
  
  portIMU=&RS485_IMU;
  portSenBus=&RS485_SENS;
  portMotor = &RS485_MOTORS;
  imu.IMU_init(portIMU);
  // Serial.println("start");
  if (imu.getIsConnected()) {
    Serial.println("IMU connect OK");

  } else {
    Serial.println("IMU connect failed");
    blink(3);
  }
  DPRINTF("RobotDriver Setup");
  Serial.flush();

  encoderPos=0;
  i_limit_yaw = 3.0;
  i_limit_pos = 50;
  pinMode(EMG_STOP,INPUT);
  pinMode(ENC_A1,INPUT);
  pinMode(ENC_B1,INPUT);
  pinMode(ENC_A2,INPUT);
  pinMode(ENC_B2,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A1), EncIntLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), EncIntRight, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A3), EncIntLift, RISING);
  Kp_yaw = 0.4;          //Yaw P-gain
  Ki_yaw = 0.1;          //Yaw I-gain
  Kd_yaw = 0.01;         //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
  desAngle = 0;
  Kp_pos = 0.010;          //Yaw P-gain
  Ki_pos = 0.006;          //Yaw I-gain
  Kd_pos = 0.002;         //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
  controlPacket[0]=0xaa;
  controlPacket[1]=0x55;
  controlPacket[2]=0x00;
  initOK=true;
  
}
void RobotDriver::gotoStandby()
{

}
void RobotDriver::posUpdate()
{
  // float distanceLeft  = (encoderPosLeft)*0.6135923151/0.05;
  // float distanceRight = (encoderPosRight)*0.6135923151/0.05;
  // float distance    = (distanceLeft+distanceRight)/2.0;
  // encoderPosLeft    = 0;
  // encoderPosRight   = 0;
  // float diff        = (distanceLeft-distanceRight);
  // float rotation    = degrees((diff/(0.5*BASE_LEN)));
  // botangle += rotation;
  // botx += sin(radians(botangle))*distance;
  // boty += cos(radians(botangle))*distance;
  // liftLevel += liftLevel;
}
void RobotDriver::calculateControlLoop()
{
  unsigned long int newTime = millis();
  int dt = newTime-timeMillis;//check dt, should be 20ms
  
  // DPRINTF(dt);
  if(dt>0&&dt<20)return;//dt minimum limit to 20 millis
  
  timeMillis=newTime;
  posUpdate();
  
  //error calculation
  float dx = desX-botx;
  float dy = desY-boty;
  DPRINT("!dx:");
  DPRINTLN(dx);
  DPRINT("!dy:");
  DPRINTLN(dy);
  float desBearing = 0;
  if(dy==0&&dx>=0) desBearing = 90;
  else if(dy==0&&dx<0) desBearing = -90;
  else desBearing = degrees( atanf(dx/dy));
  float desDistance=sqrt(dx*dx+dy*dy);
  
  float angleIMU = -imu_data.gyroyaw;
  if(angleIMU>180)angleIMU-=360;
  if(angleIMU<-180)angleIMU+=360;
  
  float desAngle = 0;
  if(desDistance>100)desAngle = desBearing;
  
  error_yaw = desBearing - botangle;
  DPRINT("desBearing:");
  DPRINTLN(desBearing);
  if(error_yaw>180)error_yaw-=360;
  if(error_yaw<-180)error_yaw+=360;
  
  integral_yaw +=  error_yaw * DT_CONTROL;
  integral_yaw = constrainVal(integral_yaw, -i_limit_yaw, i_limit_yaw); //saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) /DT_CONTROL;
  yaw_PID = (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw); //scaled by .01 to bring within -1 to 1 range
  error_yaw_prev = error_yaw;
// calculate curSpeed
  float targetSpeedRotation = 0.3*yaw_PID*(1.0-curSpeed);//high curSpeed less rotation
  targetSpeedRotation = constrainVal(targetSpeedRotation,-1,1);
  // float distance = 0;//(encoderPos-oldEncoderPos)/3.1;
  // oldEncoderPos = encoderPos;
  curSpeed =1/DT_CONTROL;
//PID speed
  error_pos = 1;
  integral_pos +=  error_pos * DT_CONTROL;
  integral_pos = constrainVal(integral_pos, -i_limit_pos, i_limit_pos); //saturate integrator to prevent unsafe buildup
  derivative_pos = (error_pos - error_pos_prev) /DT_CONTROL;
  pos_PID = .3*(Kp_pos * error_pos + Ki_pos * integral_pos + Kd_pos * derivative_pos); //scaled by .01 to bring within -1 to 1 range
  error_pos_prev = error_pos;
  float targetSpeed = pos_PID;
  targetSpeed = constrainVal(targetSpeed,-0.32,0.32);

  /*isActive = (digitalRead(EMG_STOP));
  if (isActive ==0) {   //check status
    integral_yaw = 0;
    integral_pos = 0;
    error_yaw_prev = 0;
    error_pos=0;
    
  }*/
  // update speedRobot
  // float acc = targetSpeed-desSpeed;
  // acc = constrainVal(acc,-ACC_MAX,ACC_MAX);
  float desSpeed=0;//targetSpeed;//+=acc;
  // DPRINT("targetSpeed:");
  // DPRINTLN(targetSpeed);
  // update desMotorSpeedRight 
  float acc = desSpeed-targetSpeedRotation*BASE_LEN/2.0-desMotorSpeedRight;
  acc = constrainVal(acc,-ACC_MAX*2,ACC_MAX*2);
  desMotorSpeedRight+=acc;
  desMotorSpeedRight=constrainVal(desMotorSpeedRight,-1.0,1.0);
  acc= desSpeed+targetSpeedRotation*BASE_LEN/2.0-desMotorSpeedLeft;
  acc = constrainVal(acc,-ACC_MAX*2,ACC_MAX*2);
  desMotorSpeedLeft+=acc;
  desMotorSpeedLeft=constrainVal(desMotorSpeedLeft,-1.0,1.0);
  sendControlPacket(1,desMotorSpeedRight,0);
  // DPRINT("RightSpeed:");
  // DPRINTLN(desMotorSpeedRight);
  sendControlPacket(2,desMotorSpeedLeft,0);
  // DPRINT("LeftSpeed:");
  // DPRINTLN(desMotorSpeedLeft);
  sendControlPacket(3,desMotorSpeedLift,0);

}
void RobotDriver::update()
{
  if(!initOK)return;
  // readSensBus
  while (portSenBus->available()) {
    unsigned char inputByte = portSenBus->read();
    sbus.Input(inputByte);
  }
  //read IMU
  imu.updateData();    
  if(imu.isUpdated)
  {
    imu_data = imu.getMeasurement();
  }
  if(imu.getIsConnected()==false)this->isActive=false;
  //read Motor report
  while (portMotor->available()) {
    unsigned char inputByte = portMotor->read();
    processMotorReport(inputByte);
  }
  //read imcoming command
  updateCommandBus();
  calculateControlLoop();
  
}

void RobotDriver::sendControlPacket(uint8_t id,float speed,uint8_t mode)
{
  // control left motor 
  controlPacket[2] = id;
  if(speed>0)controlPacket[4]=(0xAB);else controlPacket[4]=(0xBA);
  controlPacket[3]=((unsigned char)(abs(speed)*255));
  controlPacket[5]=mode;//Operation Mode
  controlPacket[6] = calcCS8(controlPacket,6);
  portMotor->write(controlPacket,7);
  // DPRINTLN(millis());
 
}
uint8_t reportPacket[50];
int MotorReportBuffID=0;
void RobotDriver::processMotorReport(uint8_t bytein)
{

    uint8_t lastByte = reportPacket[MotorReportBuffID];
    if((bytein==0x55)&&(lastByte==0xaa))
    {
      MotorReportBuffID = 1;
      reportPacket[0]=0xaa;
      reportPacket[1]=0x55;
    }
    else
    {
      MotorReportBuffID++;
      if (MotorReportBuffID >= 49)MotorReportBuffID = 0;
      reportPacket[MotorReportBuffID] = bytein;
    }
    if (MotorReportBuffID > 6) {
      if (reportPacket[1] == 0x55)
        if (reportPacket[0] == 0xaa) {
          if(reportPacket[2]== 0xf1)
          {

          }
        }
      // MotorReportBuffID=0;
    }
    if (MotorReportBuffID >=19) 
    if (reportPacket[0] == 0xaa) 
    if (reportPacket[1] == 0x55)
    if (reportPacket[2] == 0xf1)
    if (reportPacket[4] == 0xff)
    if (reportPacket[5] == 0x03)
    {
      DPRINTLN("\npacketOK");
      int x =     (reportPacket[6]<<24)  + (reportPacket[7]<<16)  + (reportPacket[8]<<8)  +   reportPacket[9] ;
      int y =     (reportPacket[10]<<24) + (reportPacket[11]<<16) + (reportPacket[12]<<8) +   reportPacket[13] ;
      float angle = (reportPacket[14]<<8) +   reportPacket[15] ;
      angle/=10.0;
      DPRINT("report x:");
      DPRINTLN(x);
      DPRINT("report y:");
      DPRINTLN(y);
      DPRINT("report angle:");
      DPRINTLN(angle);
      botangle = angle;
      
    }



}
