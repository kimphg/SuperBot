
#include "motorBLVPWM.h"
#include "mti.h"

class CamData
{
  public:
  CamData(){}
  float getTagBearing()
  {
    return 0;
  }
  float x,y;
  float angle;
  int tagid;
  int connectCount;
} ;
CamData camh7data;
class Robot
{
  public:
  Robot(){
    botangle=0;
  }

  void updateSensors();

  void updateData();

  void updateControl();

  void gotoStandby();

  void gotoFailsafe();

  bool isActive();
  float getRotSpeed();
  private:
  int botState;
  motorBLVPWM mMotor;
  float imuAngle;
  float botx,posy;
  float botangle;
  int CurTagid;
  IMU_driver imuMti;
} ;
Robot mRobot;