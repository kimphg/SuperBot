#include "common.h"
class CamData {
public:
  CamData() {}
  float getTagBearing() {
    return angle;
  }
  float x, y;
  float angle;
  int tagid;
  int connectCount;
};

class SenBusDriver {
public:
  SenBusDriver() {
    sensBusBuffi = 0;
  }
  CamData camh7data;
  bool Input(unsigned char inputByte);
  void processCamera(String inputStr);
  
float tagAngle=0;
  private:
  
  String inputString;
  unsigned char sensBusBuff[100];
  unsigned char sensBusBuffo;
  int h7ConnectCount = 0;
  int sensBusBuffi = 0;

  
};