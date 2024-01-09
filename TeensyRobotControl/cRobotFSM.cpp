#include "cRobotFSM.h"

void RobotDriver::processCommand(String command)
{
  int dataLen = command.length();
  if(dataLen<8)return;
  if(command.startsWith("$COM,m,"))
  {
    float des = command.substring(7, dataLen - 1).toFloat();
    DEBUG_TELEMETRY.print("Motion Command des:");
    DEBUG_TELEMETRY.print(des);
  }


}

RobotDriver::RobotDriver(Stream &sportIMU, Stream &sportSenBus)
{
  portIMU=&sportIMU;
  portSenBus=&sportSenBus;
  imu.IMU_init(sportIMU);
  Serial.println("start");
  if (imu.getIsConnected()) {
    Serial.println("IMU connect OK");

  } else {
    Serial.println("IMU connect failed");
    indicateErrorLed(3);
  }
  Serial.flush();
}
void RobotDriver::gotoStandby()
{
}

void RobotDriver::update()
{
  // readSensBus
  while (portSenBus->available()) {
    unsigned char inputByte = portSenBus->read();
    sbus.Input(inputByte);
  }
  imu.updateData();    //read IMU
}
