
#include "motorBLVPWM.h"
#define DT_CONTROL 0.02 //50hz control loop
#define ACC_MAX 0.07*DT_CONTROL
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
    outputSpeed1=0;
    outputSpeed2=0;
    targetSpeed = 0;
    initMotor1();
    initMotor2();
    timeMillis = millis();
}
void motorBLVPWM::update()
{
    unsigned long int newTime = millis();
    int dt = newTime-timeMillis;
    timeMillis=newTime;
    if(dt<1)return;
    speedMotor1 = speed_pulse_counter1/dt;
    speed_pulse_counter1=0;
    speedMotor2 = speed_pulse_counter2/dt;
    speed_pulse_counter2=0;
    if(outputSpeed1<targetSpeed)outputSpeed1+=ACC_MAX;
    if(outputSpeed2>targetSpeed)outputSpeed2-=ACC_MAX;
}
void motorBLVPWM::initMotor1()
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
void motorBLVPWM::initMotor2()
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
void motorBLVPWM::setMotorSpeed2(float speed)
{
  if(speed>=0)
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
void motorBLVPWM::setMotorSpeed1(float speed)
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