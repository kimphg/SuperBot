#include "Arduino.h"

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
  bool Input(unsigned char inputByte, float roll,float pitch);
  bool processCamera(String inputStr, float roll,float pitch);
  float tagAngle=0;
  float tagX=0,tagY=0;
  float dRoll=0,dPitch=0;
  int tagID;
  private:
  int lastTagID=-1;
  String inputString;
  unsigned char sensBusBuff[100];
  unsigned char sensBusBuffo;
  int h7ConnectCount = 0;
  int sensBusBuffi = 0;

  
};