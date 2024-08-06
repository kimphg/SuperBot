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
    // sensBusBuffi = 0;
  }
  CamData camh7data;
  int syncLossCount = 0;
  int Input(unsigned char inputByte);
  int processCamera(String inputStr);
  int processCommand(String command);
  float tagAngle=0;
  float tagX=0,tagY=0;
  float        desX = 0;
  float      desY = 0;
  String commandBuff;
  int tagID;
  private:
  int lastTagID=-1;
  String inputString;
  // unsigned char sensBusBuff[100];
  // unsigned char sensBusBuffo;
  int h7ConnectCount = 0;
  // int sensBusBuffi = 0;

  
};