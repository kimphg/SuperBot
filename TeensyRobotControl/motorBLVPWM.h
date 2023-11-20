
#ifndef MOTORBLVPWM
#define MOTORBLVPWM
#define M1_FWD 29
#define M1_REV 28
#define M1_STOP_MODE 27
#define M1_M0 26
#define M1_MB_FREE 9
#define M1_PWM 8
#define M1_IN_SPEED 30

#define M2_FWD 7
#define M2_REV 6
#define M2_STOP_MODE 5
#define M2_M0 4
#define M2_MB_FREE 3
#define M2_PWM 2
#define M2_IN_SPEED 24
#include <Arduino.h>
#define DT_CONTROL 0.02 //50hz control loop
#define ACC_MAX 0.07/DT_CONTROL
#define BASE_LEN 0.5
/// @brief Class for BLV motor control with DB15 connector and PWM speed control
class motorBLVPWM {
 public:

    motorBLVPWM();
    void update();
    
    unsigned long int timeMillis;
    float speedLeftFeedback,speedRightFeedback;
    float speedLeft,speedRight;
    float speedRobot;
    void SetControlValue(float speed,float rotationSpeed);
private:
     float targetSpeed,    targetSpeedRotation ;
    void initMotorLeft();
    void initMotorRight( );
    void setMotorLeft(float speed);
    void setMotorRight(float speed);
};
#endif