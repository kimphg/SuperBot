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
#define OUTPUT_1 2
#define OUTPUT_2 3
#define OUTPUT_3 4
#define OUTPUT_4 5
#define DT_CONTROL 0.02 //50hz control loop
#define ACC_MAX 0.05/DT_CONTROL
#define BASE_LEN 0.5
class RobotDriver
{
  public:
  RobotDriver();
  void processCommand(String command);
  void update();
void calculateControlLoop();
  private:
void sendControlPacket();
  IMUData imu_data;
  int encoderPos=0;
  bool isActive=false;
  float i_limit_yaw = 3.0; 
  float i_limit_pos = 30.0; 
  float Kp_yaw ,Ki_yaw , Kd_yaw ;   
  float Kp_pos ,Ki_pos, Kd_pos ;  
  float error_pos, error_pos_prev, integral_pos,  derivative_pos, pos_PID = 0;
  float error_yaw, error_yaw_prev, integral_yaw,  derivative_yaw, yaw_PID = 0;
  void gotoStandby();
  
  IMU_driver imu;
  Stream *portIMU;
  Stream *portSenBus;
  Stream *portMotor;
unsigned long int timeMillis=0;
  float desMotorSpeedLeft=0,desMotorSpeedRight=0,desMotorSpeedLift=0;
  int botState;
  SenBusDriver sbus;
  float botx,boty;
  float botangle;
  float desPos=0;
  float desAngle=0;
  float curSpeed=0;
  int CurTagid;
} ;
