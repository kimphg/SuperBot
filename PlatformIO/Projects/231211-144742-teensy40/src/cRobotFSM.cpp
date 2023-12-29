#include "cRobotFSM.h"
void Robot::updateSensors(){
  
  IMUData measure = imuMti.getMeasurement();
  // AccX = measure.accY;
  // AccY = measure.accY;
  // AccZ = measure.accZ;
  // GyroX = measure.gyroX*57.2958;
  // GyroY = measure.gyroY*57.2958;
  // GyroZ = measure.gyroZ*57.2958;
  // yaw_IMU = measure.gyroyaw*57.2958;
  imuAngle = measure.gyroyaw;
  // Serial.print(yaw_IMU);
  // Serial.print(" ");
  // Serial.print(measure.yaw-170.3);
  // Serial.print(" ");
  // Serial.println(imu.yawCalcMode-100);
}
void Robot::updateData()
{
    imuMti.updateData();//read IMU
}
void Robot::updateControl()
{
    
}
void Robot::gotoStandby()
{

}
void Robot::gotoFailsafe()
{

}
bool Robot::isActive()
{
    if(botState==0)return false;
    return true;
}

float Robot::getRotSpeed()
{
    return mMotor.getRotSpeed();
    // return 0.0f;
}
