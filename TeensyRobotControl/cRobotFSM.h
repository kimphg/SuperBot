#include "senbusdriver.h"
#include <Arduino.h>
#include "mti.h"
#include "common.h"
class RobotDriver
{
  public:
  RobotDriver(Stream &sportIMU,Stream &sportSenBus,Stream &sportMotor);
  void processCommand(String command);
  void update();

  private:
  void gotoStandby();
  void executeMotion();
  IMU_driver imu;
  Stream *portIMU;
  Stream *portSenBus;
  Stream *portMotor;
  int botState;
  SenBusDriver sbus;
  float botx,posy;
  float botangle;
  int CurTagid;
} ;
