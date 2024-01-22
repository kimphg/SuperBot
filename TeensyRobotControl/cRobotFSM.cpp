
#include "cRobotFSM.h"
long int encoderPos=0;
long int oldEncoderPos = encoderPos;
int liftLevel = 0;
#define CONTROL_LEN 10
unsigned char controlPacket[CONTROL_LEN];
void EncIntLeft()
{
  if(digitalRead(ENC_B1)>0)
  encoderPos++;
  else encoderPos--;
  
}
void EncIntRight()
{
  if(digitalRead(ENC_B2)>0)
  encoderPos--;
  else encoderPos++;
  
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
  std::vector<String> tokens = splitString(command);
  if(tokens.size()>=2)
  {
    if(tokens[0].equals("$COM"))
    {
      if(tokens[1].equals("m"))
      {
          DPRINTF("Motion command start:");
          float comX = (tokens[2].toFloat());
          float comY = tokens[3].toFloat();
          DPRINTLN(comX);
          DPRINTLN(comY);
      }
    }
  }
  else {DPRINTLNF("Command too short");}

}

RobotDriver::RobotDriver()
{
  RS485_IMU.begin(921600);//IMU
  RS485_SENS.begin(1000000);//sens bus
  RS485_MOTORS.begin(115200);
  portIMU=&RS485_IMU;
  portSenBus=&RS485_SENS;
  portMotor = &RS485_MOTORS;
  imu.IMU_init(portIMU);
  Serial.println("start");
  if (imu.getIsConnected()) {
    Serial.println("IMU connect OK");

  } else {
    Serial.println("IMU connect failed");
    indicateErrorLed(3);
  }
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
  controlPacket[0]=0x0a;
  controlPacket[1]=0x55;
  controlPacket[2]=0x00;
  
}
void RobotDriver::gotoStandby()
{

}
void RobotDriver::calculateControlLoop()
{
  unsigned long int newTime = millis();
  int dt = newTime-timeMillis;//check dt, should be 20ms
  
  // DPRINTF(dt);
  if(dt>0&&dt<20)return;//dt minimum limit to 20 millis
  timeMillis=newTime;
  float angleIMU = imu_data.gyroyaw;
  if(angleIMU>180)angleIMU-=360;
  if(angleIMU<-180)angleIMU+=360;

  error_yaw = desAngle - angleIMU;
  if(error_yaw>180)error_yaw-=360;
  if(error_yaw<-180)error_yaw+=360;
  
  integral_yaw +=  error_yaw * DT_CONTROL;
  integral_yaw = constrainVal(integral_yaw, -i_limit_yaw, i_limit_yaw); //saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) /DT_CONTROL;
  yaw_PID = (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw); //scaled by .01 to bring within -1 to 1 range
  error_yaw_prev = error_yaw;
// calculate curSpeed
  float targetSpeedRotation = 0.3*yaw_PID*(1.0-curSpeed);
  targetSpeedRotation = constrainVal(targetSpeedRotation,-1,1);
  float distance = (encoderPos-oldEncoderPos)/3.1;
  oldEncoderPos = encoderPos;
  curSpeed =distance/DT_CONTROL;
//PID speed
  error_pos = desPos;
  integral_pos +=  error_pos * DT_CONTROL;
  integral_pos = constrainVal(integral_pos, -i_limit_pos, i_limit_pos); //saturate integrator to prevent unsafe buildup
  derivative_pos = (error_pos - error_pos_prev) /DT_CONTROL;
  pos_PID = .3*(Kp_pos * error_pos + Ki_pos * integral_pos + Kd_pos * derivative_pos); //scaled by .01 to bring within -1 to 1 range
  error_pos_prev = error_pos;
  float targetSpeed = pos_PID;
  targetSpeed = constrainVal(targetSpeed,-0.32,0.32);

  isActive = (digitalRead(EMG_STOP));
  if (isActive ==0) {   //check status
    integral_yaw = 0;
    integral_pos = 0;
    error_yaw_prev = 0;
    error_pos=0;
    return;
  }
  // update speedRobot
  // float acc = targetSpeed-desSpeed;
  // acc = constrainVal(acc,-ACC_MAX,ACC_MAX);
  float desSpeed=targetSpeed;//+=acc;
  // update desMotorSpeedRight 
  float acc = desSpeed-targetSpeedRotation*BASE_LEN/2.0-desMotorSpeedRight;
  acc = constrainVal(acc,-ACC_MAX*2,ACC_MAX*2);
  desMotorSpeedRight+=acc;
  desMotorSpeedRight=constrainVal(desMotorSpeedRight,0,1.0);
  acc= desSpeed+targetSpeedRotation*BASE_LEN/2.0-desMotorSpeedLeft;
  acc = constrainVal(acc,-ACC_MAX*2,ACC_MAX*2);
  desMotorSpeedLeft+=acc;
  desMotorSpeedLeft=constrainVal(desMotorSpeedLeft,0,1.0);
  sendControlPacket();

}
void RobotDriver::update()
{
  // readSensBus
  while (portSenBus->available()) {
    unsigned char inputByte = portSenBus->read();
    sbus.Input(inputByte);
  }
  imu.updateData();    //read IMU
  if(imu.isUpdated)
  {
    imu_data = imu.getMeasurement();
  }
  if(imu.getIsConnected()==false)this->isActive=false;
  
}

void RobotDriver::sendControlPacket()
{
  // control left motor 
  controlPacket[2] = 0x01;
  if(desMotorSpeedLeft>0)controlPacket[4]=(0xAB);else controlPacket[4]=(0xBA);
  controlPacket[3]=((unsigned char)(abs(desMotorSpeedLeft)*255));
  controlPacket[5]=0x00;//Operation Mode
  int cs = 0;
  for (int i = 0; i < 6; i++) {
    cs ^= controlPacket[i];
  }
  controlPacket[6] = cs;
  portMotor->write(controlPacket,7);
  //portMotor->flush();
  // control right motor 
  controlPacket[2] = 0x02;
  if(desMotorSpeedRight>0)controlPacket[4]=(0xAB);else controlPacket[4]=(0xBA);
  controlPacket[3]=((unsigned char)(abs(desMotorSpeedRight)*255));
  controlPacket[5]=0x00;//Operation Mode
  int cs = 0;
  for (int i = 0; i < 6; i++) {
    cs ^= controlPacket[i];
  }
  controlPacket[6] = cs;
  portMotor->write(controlPacket,7);
  // control lift motor 
  controlPacket[2] = 0x03;
  if(desMotorSpeedLift>0)controlPacket[4]=(0xAB);else controlPacket[4]=(0xBA);
  controlPacket[3]=((unsigned char)(abs(desMotorSpeedLift)*255));
  controlPacket[5]=0x00;//Operation Mode
  int cs = 0;
  for (int i = 0; i < 6; i++) {
    cs ^= controlPacket[i];
  }
  controlPacket[6] = cs;

  portMotor->write(controlPacket,7);
}