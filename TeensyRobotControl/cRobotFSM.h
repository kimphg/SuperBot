#include "senbusdriver.h"
#include <Arduino.h>
#include "mti.h"
#include "common.h"
#define EMG_STOP 32
#define ENC_A1 6
#define ENC_B1 9
#define ENC_A2 10
#define ENC_B2 11
#define ENC_A3 12
#define ENC_B3 23
#define ENC_Z3 5
#define OUTPUT_1 2
#define OUTPUT_2 3
#define OUTPUT_3 4
#define OUTPUT_4 5
#define STEP_PULSE OUTPUT_1
#define STEP_DIR OUTPUT_2
#define DT_CONTROL 0.02 //50hz control loop
#define DT_POS_UPDATE 0.02 //20hz  loop
#define BASE_LEN 0.45
#define COMMAND_LEN_MAX 100
#define MODE_STANDBY 0
#define MODE_MOVE 1
#define MODE_ROTATE 2
#define MODE_LIFT 3
#define MAX_MOTION_SPEED 1.3
#define LIFT_PPR 1485.0
#define STEP_PPR 14688
struct RobotParam
{
  String paramName;
  float paraValue;
};
struct FloorTag
{
  int id;
  int x,y;
};
#define PARAM_ID_YAW_KP 0


class RobotDriver
{
  public:
      
      RobotDriver();
      int getclosestTag(float x, float y);
      int closestID=-1;
      int closestIDDist=0;
      int palletAligned=0;
      void reportPPU();
      float loadParam(String id, float defaultValue);
      void setParam(String id, float value);
      void processCommand(String command);
      void update();
      void gotoMode(int mode);
      void sendSyncPacket();
      void loadParams();
  void motorUpdate();
  private:
  int lastSyncSec=0;
  int syncLossCount=0;
  float maxBotSpeed = 0.6;
  float maxBotAcc = 0.005;
  float maxBotRotSpd = 1.5;//radian/s
  float diffAngle = 0;
  int debugCounter=0;
  unsigned long int curTime =0;;
  float botRotationSpeed = 0;
  bool isLiftMinPos = false;
  bool isLiftMaxPos = false;
  bool paramchanged = true;
  float desRotSpd=0;
  float  desDistance = 0;
  float desBearing = 0;
  int curLiftStep = 0;
  int lastFloorTagid = -1;
  int lastFloorYawTagid = -1;
  int minLiftStep = -10000;
  int stepFreq = 1;
  int desLiftStep = 0;
  bool initOK = false;
  uint8_t cur_lift_stat=0;
  int newliftComm = 0;
  void report_pos_to_PPU();
  void loopRotate();
  void liftStabilize();
  void updateCommandBus();
  void processCommandBytes();
  void sendPPUack(uint8_t commandCode);
  void sendControlPacket(uint8_t id, float speed, uint8_t mode);
  void processMotorReport(uint8_t bytein);
  int  processFrontBoard(String inputStr);
  void checkMinMax();
  void loopMove(float maxDistance=25);
  void loopLift();
  void stepOutput(int dir);
  void loopStandby();
  IMUData imu_data;
  float botCameraCorrection=0;
  void liftStabilizeBLV();
  int bot_mode = MODE_STANDBY;
  int stillCount=0;
  float palletAngle =0;
  bool isActive=false;
  float botAngleAcc = 100;
  float i_limit_yaw = 10.0; 
  float i_limit_pos = 150.0; 
  float Kp_yaw ,Ki_yaw , Kd_yaw ;   
  float Kp_pos ,Ki_pos, Kd_pos ;  
  float Kp_lift ,Ki_lift, Kd_lift,Ks_lift ;  
  float error_pos, error_pos_prev, integral_pos=0,  derivative_pos, pos_PID = 0;
  float error_yaw, error_yaw_prev, integral_yaw=0,  derivative_yaw, yaw_PID = 0;
  void gotoStandby();
  void posUpdate();
  float calcPIDyaw(float targetAngle);
  void setSpeedRight(float value);
  void setSpeedLeft(float value);
  float calcPIDPos(float epos);
  void DebugReport();
  void controlLoop();
  IMU_driver imu;
  Stream *portIMU;
  Stream *portSenBus;
  Stream *portMotor;
unsigned long int lastLoopMillis=0;
// unsigned long int lastPosUpdateMillis=0;
  float desMotSpdL=0,desMotSpdR=0;
  int botState = 0;
  SenBusDriver sbus;
  float desSpeed;
  float liftAngleError = 0;
  bool liftLevelMinDefined = false;
  float botx = 0,boty = 0;
  float botangle = 0;
  float desX=0,desY=0;
  float desAngle=0;
  float curSpeed=0;
  float curSpeedL=0;
  float curSpeedR=0;
  float curSpeedLift=0;
  float maxLiftAcc = 0.001;
  float liftStabDelay =15;
  float desLiftSpeed = 0;
  float desliftAngle = 0;
  int liftLevelDown =  LIFT_PPR/2.0;
  int liftLevelUp = LIFT_PPR*6.5;
  float liftLevelAngle =0.0;
  float liftAngle =0.0;
  int CurTagid;
} ;

