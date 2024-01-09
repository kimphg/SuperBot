#include "senbusdriver.h"
#include <Arduino.h>
#include "mti.h"
#include "common.h"
class RobotDriver
{
  public:
  RobotDriver(Stream &sportIMU,Stream &sportSenBus);
  void processCommand(String command);
  void gotoStandby();
  void update();
  private:
  IMU_driver imu;
  Stream *portIMU;
  Stream *portSenBus;
  int botState;
  SenBusDriver sbus;
  float botx,posy;
  float botangle;
  int CurTagid;
} ;
