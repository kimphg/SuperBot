
#pragma once
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
/// @brief Class for BLV motor control with DB15 connector and PWM speed control
class motorBLVPWM {
 public:

    motorBLVPWM();
    void update();
    void initMotor1();
    void initMotor2( );
    unsigned long int timeMillis;
    float speedMotor1,speedMotor2;
    float outputSpeed1,outputSpeed2;
    float targetSpeed;
private:
    
};