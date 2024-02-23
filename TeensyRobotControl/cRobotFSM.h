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
#define DT_CONTROL 0.02 //50hz control loop
#define ACC_MAX 0.005/DT_CONTROL
#define BASE_LEN 0.45
#define COMMAND_LEN_MAX 100
#define MODE_STANDBY 0
#define MODE_MOVE 1
#define MODE_ROTATE 2
#define MODE_LIFT 3
#define MAX_MOTION_SPEED 1.3
class RobotDriver
{
  public:
      
      RobotDriver();
      void processCommand(String command);
      void update();

      void gotoMode(int mode);

      

      void sendSyncPacket();
      

  private:
  float maxBotSpeed = 0.3;
  float maxBotAcc = 0.005/DT_CONTROL;
  float maxBotRotSpd = 1.0;//radian/s
  float CalcPIDyaw(float targetAngle);
  float CalcPIDPos(float epos);
  int debugCounter=0;
  unsigned long int curTime =0;;
  float botRotationSpeed = 0;
  bool isLiftMinPos = false;
  bool isLiftMaxPos = false;
  float desRotSpd=0;
  float  desDistance = 0;
  float desBearing = 0;
  bool initOK = false;
  
  void loopRotate();
  void liftStabilize();
  void updateCommandBus();
  void sendControlPacket(uint8_t id,float speed,uint8_t mode);
  void processMotorReport(uint8_t bytein);
  void loopMove();
  void loopLift();
  void loopStandby();
  IMUData imu_data;
  int bot_mode = MODE_STANDBY;
  int stillCount=0;
  
  bool isActive=false;
  float i_limit_yaw = 5.0; 
  float i_limit_pos = 150.0; 
  float Kp_yaw ,Ki_yaw , Kd_yaw ;   
  float Kp_pos ,Ki_pos, Kd_pos ;  
  float error_pos, error_pos_prev, integral_pos=0,  derivative_pos, pos_PID = 0;
  float error_yaw, error_yaw_prev, integral_yaw=0,  derivative_yaw, yaw_PID = 0;
  void gotoStandby();
  void posUpdate();
  void DebugReport();
  void controlLoop();
  IMU_driver imu;
  Stream *portIMU;
  Stream *portSenBus;
  Stream *portMotor;
unsigned long int lastLoopMillis=0;
unsigned long int lastPosUpdateMillis=0;
  float desMotSpdL=0,desMotSpdR=0,desMotorSpeedLift=0;
  int botState = 0;
  SenBusDriver sbus;
  float botx = 0,boty = 0;
  float botangle = 0;
  float desX=0,desY=0;
  float desAngle=0;
  float curSpeed=0;
  float curSpeedL=0;
  float curSpeedR=0;
  float curSpeedLift=0;
  float desLiftSpeed = 0;
  int liftLevelA0 = 340;
  float liftLevelAngle =0.0;
  float liftAngle =0.0;
  int CurTagid;
} ;
