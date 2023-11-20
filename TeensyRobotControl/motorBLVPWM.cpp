
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
}

void motorBLVPWM::update()
{
    // Serial.println("motor update");
    unsigned long int newTime = millis();
    int dt = newTime-timeMillis;//check dt, should be 20ms
    timeMillis=newTime;
    // Serial.println(dt);
    if(dt<1)return;//dt too small
    //
    speedLeftFeedback = speed_pulse_counter1/dt;
    speed_pulse_counter1=0;
    speedRightFeedback = speed_pulse_counter2/dt;
    speed_pulse_counter2=0;
    // update speedRobot
    float acc = targetSpeed-speedRobot;
    constrain(acc,-ACC_MAX,ACC_MAX);
    speedRobot+=acc;
    // update speedRight speedLeft
    speedRight = speedRobot-targetSpeedRotation*BASE_LEN/2.0;
    speedLeft = speedRobot+targetSpeedRotation*BASE_LEN/2.0;
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
    delay(10);
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
    delay(10);
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