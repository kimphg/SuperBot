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
  CamData(){}
  int botState;
  void gotoStandby()
  {

  }
  float botx,posy;
  float botangle;
  int CurTagid;
} ;
Robot mRobot;