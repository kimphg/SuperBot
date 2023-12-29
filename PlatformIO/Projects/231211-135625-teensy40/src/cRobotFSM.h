#include "common.h"
class CamData
{
  public:
  CamData(){}
  float getTagBearing()
  {
    
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
  Robot(){}
  int botState;
  int h7ConnectCount=0;
  void gotoStandby()
  {

  }
  
  float botx,posy;
  float botangle;
  int CurTagid;
} ;
Robot mRobot;