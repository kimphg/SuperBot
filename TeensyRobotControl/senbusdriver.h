#include "common.h"
class CamData {
public:
  CamData() {
    updatetime=millis();
  }
  void setValue(int id,float x,float y,float angle){
    tagAngle=angle;
    if(tagID == id){stable++;if(stable>10)stable=10;}
    else
    {
      stable=0;
      tagID = id;
    }
    tagX = x;
    tagY = y;
    updatetime=millis();
  }
  int getage()
  {
    return (millis()-updatetime);
  }
  float tagAngle=0;
  float tagX=0,tagY=0;
  int tagID=-1;
  int stable=0;
  int updatetime=0;
};

class SenBusDriver {
public:
  SenBusDriver() {
    // sensBusBuffi = 0;
  }
  CamData cambot,camtop;
  int fb_warning_level = 0;
  int warning_repeated=0;
  int syncLossCount = 0;

  int Input(unsigned char inputByte);
  int processCamera(String inputStr);
  int processFrontBoard(String command);
  int processCommand(String command);
  int processCameraTop(String inputStr);

  float        desX = 0;
  float      desY = 0;
  String commandBuff;
  private:
  String inputString;
  // unsigned char sensBusBuff[100];
  // unsigned char sensBusBuffo;
  int h7ConnectCount = 0;
  // int sensBusBuffi = 0;

  
};