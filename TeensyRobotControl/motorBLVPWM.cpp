
#include "motorBLVPWM.h"

unsigned int speed_pulse_counter1,speed_pulse_counter2;
void readSpeed1()
{
    speed_pulse_counter1++;

}
void readSpeed2()
{
    speed_pulse_counter2++;

}
motorBLVPWM::motorBLVPWM()
{
    speed_pulse_counter1=0;
    speed_pulse_counter2=0;
    speedLeft=0;
    speedRight=0;
    speedRobot = 0;
    targetSpeed = 0;
    initMotorLeft();
    initMotorRight();
    timeMillis = millis();
    setMotorLeft(0.0);  
    setMotorRight(0.0); 
    range=0;
    Kp_yaw = 0.6;           //Yaw P-gain
    Ki_yaw = 0.15;          //Yaw I-gain
    Kd_yaw = 0.005;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
}


void motorBLVPWM::update(float angleIMU)
{
    error_yaw = yaw_des - angleIMU;
    if(error_yaw>180)error_yaw-=360;
    if(error_yaw<-180)error_yaw+=360;
    integral_yaw = integral_yaw_prev + error_yaw * dt/1000000.0;
    if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
      integral_yaw = 0;
    }


    integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
    derivative_yaw = (error_yaw - error_yaw_prev) / dt*10000000.0;
    yaw_PID = .3 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw); //scaled by .01 to bring within -1 to 1 range
    error_yaw_prev = error_yaw;
    integral_yaw_prev = integral_yaw;
    targetSpeedRotation = yaw_PID;
    // Serial.println("motor update");
    unsigned long int newTime = millis();
    int dt = newTime-timeMillis;//check dt, should be 20ms
    timeMillis=newTime;
    // Serial.println(dt);
    if(dt<1)return;//dt too small
    //
    if(speedLeft>0)speedLeftFeedback = speed_pulse_counter1/float(dt)*1000;
    else speedLeftFeedback = -speed_pulse_counter1/float(dt)*1000;
    speed_pulse_counter1=0;
    if(speedRight>0)speedRightFeedback = speed_pulse_counter2/float(dt)*1000;
    else speedRightFeedback = -speed_pulse_counter2/float(dt)*1000;
    speed_pulse_counter2=0;
    range+=(speedLeftFeedback+speedRightFeedback)/2;
    // update speedRobot
    float acc = targetSpeed-speedRobot;
    constrain(acc,-ACC_MAX,ACC_MAX);
    speedRobot+=acc;
    // update speedRight speedLeft
    acc = speedRobot-targetSpeedRotation*BASE_LEN/2.0-speedRight;
    constrain(acc,-ACC_MAX,ACC_MAX);
    speedRight+=acc;
    acc= speedRobot+targetSpeedRotation*BASE_LEN/2.0-speedLeft;
    constrain(acc,-ACC_MAX,ACC_MAX);
    speedLeft+=acc;
    setMotorLeft(speedLeft);
    setMotorRight(speedRight);
    
    
}
void motorBLVPWM::SetControlValue(float speed,float rotationSpeed)
{
     targetSpeed = speed;//-rotationSpeed*BASE_LEN/2.0;
     targetSpeedRotation = -rotationSpeed;// speed+rotationSpeed*BASE_LEN/2.0;
}
void motorBLVPWM::initMotorLeft()
{
    pinMode(M1_FWD,OUTPUT);
    pinMode(M1_REV,OUTPUT);
    pinMode(M1_STOP_MODE,OUTPUT);
    pinMode(M1_M0,OUTPUT);
    pinMode(M1_MB_FREE,OUTPUT);
    pinMode(M1_PWM,OUTPUT);
    pinMode(M1_IN_SPEED,INPUT);
    digitalWrite(M1_FWD,LOW);//Deceleration stop 
    digitalWrite(M1_REV,LOW);//Deceleration stop 
    digitalWrite(M1_STOP_MODE,HIGH);//Deceleration stop 
    digitalWrite(M1_MB_FREE,HIGH);//electromagnetic brake released
    digitalWrite(M1_M0,HIGH);//electromagnetic brake released
    analogWrite(M1_PWM,256);
    attachInterrupt(digitalPinToInterrupt(M1_IN_SPEED), readSpeed1, RISING);
}
void motorBLVPWM::initMotorRight()
{
    pinMode(M2_FWD,OUTPUT);
    pinMode(M2_REV,OUTPUT);
    pinMode(M2_STOP_MODE,OUTPUT);
    pinMode(M2_M0,OUTPUT);
    pinMode(M2_MB_FREE,OUTPUT);
    pinMode(M2_PWM,OUTPUT);
    pinMode(M2_IN_SPEED,INPUT);
    digitalWrite(M2_FWD,LOW);//Deceleration stop 
    digitalWrite(M2_REV,LOW);//Deceleration stop 
    digitalWrite(M2_STOP_MODE,HIGH);//Deceleration stop 
    digitalWrite(M2_MB_FREE,HIGH);//electromagnetic brake released
    digitalWrite(M2_M0,HIGH);//electromagnetic brake released
    analogWrite(M2_PWM,256);
    attachInterrupt(digitalPinToInterrupt(M2_IN_SPEED), readSpeed2, RISING);
}
void motorBLVPWM::setMotorRight(float speed)
{
  if(speed<0)
  {  
    digitalWrite(M2_FWD,HIGH);
    digitalWrite(M2_REV,LOW);}
  else 
  {  
    digitalWrite(M2_FWD,LOW);
    digitalWrite(M2_REV,HIGH);
    }
  float outspeed=abs(speed);
  if(outspeed>1)outspeed=1;
  int pwm_value = (1.0-outspeed)*256;
  analogWrite(M2_PWM,pwm_value);
}
void motorBLVPWM::setMotorLeft(float speed)
{
  if(speed>=0)
  {  
    digitalWrite(M1_FWD,HIGH);
    digitalWrite(M1_REV,LOW);}
  else 
  {  
    digitalWrite(M1_FWD,LOW);
    digitalWrite(M1_REV,HIGH);
    }
  float outspeed=abs(speed);
  if(outspeed>1)outspeed=1;
  int pwm_value = (1.0-outspeed)*256;
  analogWrite(M1_PWM,pwm_value);
}