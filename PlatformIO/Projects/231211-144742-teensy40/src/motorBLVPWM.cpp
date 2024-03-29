
#include "motorBLVPWM.h"
#define EMG_STOP 32
#define ENC_A1 13
#define ENC_B1 11
#define ENC_A2 12
#define ENC_B2 10
#define Debug Serial
int encoderPos=0;
float constrainVal(float input,float min, float max)
{
  if(input<min)return min;
  if(input>max)return max;
  return input;
}
unsigned int speed_pulse_counter1,speed_pulse_counter2;
void EncRight()
{
  if(digitalRead(ENC_B1)>0)
  encoderPos++;
  else encoderPos--;
  
}
void EncLeft()
{
  if(digitalRead(ENC_B2)>0)
  encoderPos--;
  else encoderPos++;
  
}
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
    encoderPos=0;
    i_limit_yaw = 3.0;
    i_limit_pos = 50;
    speed_pulse_counter1=0;
    speed_pulse_counter2=0;
    speedLeft=0;
    speedRight=0;
    speedRobot = 0;
    targetSpeed = 0;
    initMotorLeft();
    initMotorRight();
    // delay(5);
    timeMillis = millis();
    setMotorLeft(0.0);  
    setMotorRight(0.0); 
    pinMode(EMG_STOP,INPUT);
    pinMode(ENC_A1,INPUT);
    pinMode(ENC_B1,INPUT);
    pinMode(ENC_A2,INPUT);
    pinMode(ENC_B2,INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC_A1), EncRight, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_A2), EncLeft, RISING);
    encoderPos=0;
    posYOld=0;
    Kp_yaw = 0.4;          //Yaw P-gain
    Ki_yaw = 0.1;          //Yaw I-gain
    Kd_yaw = 0.01;         //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
    yaw_des= 0;
    Kp_pos = 0.010;          //Yaw P-gain
    Ki_pos = 0.006;          //Yaw I-gain
    Kd_pos = 0.002;         //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
    pos_des= 0;
}

void motorBLVPWM::resetPosition()
{
  encoderPos=0;
  posYOld=0;
}

void motorBLVPWM::update(float angleIMU)
{
       
    unsigned long int newTime = millis();
    int dt = newTime-timeMillis;//check dt, should be 20ms
    timeMillis=newTime;
    // Debug.println(dt);
    if(dt<1)return;//dt too small

    error_yaw = yaw_des - angleIMU;
    if(error_yaw>180)error_yaw-=360;
    if(error_yaw<-180)error_yaw+=360;
    integral_yaw +=  error_yaw * DT_CONTROL;
    integral_yaw = constrainVal(integral_yaw, -i_limit_yaw, i_limit_yaw); //saturate integrator to prevent unsafe buildup
    derivative_yaw = (error_yaw - error_yaw_prev) /DT_CONTROL;
    yaw_PID = (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw); //scaled by .01 to bring within -1 to 1 range
    error_yaw_prev = error_yaw;

    targetSpeedRotation = 0.3*yaw_PID*(1.0-curSpeed);
    targetSpeedRotation = constrainVal(targetSpeedRotation,-1,1);

    
    posY = encoderPos/3.1;
    curSpeed += 0.1*((posY-posYOld)/DT_CONTROL-curSpeed);
    posYOld = posY;
    error_pos = pos_des - posY;
    integral_pos +=  error_pos * DT_CONTROL;
    integral_pos = constrainVal(integral_pos, -i_limit_pos, i_limit_pos); //saturate integrator to prevent unsafe buildup
    derivative_pos = (error_pos - error_pos_prev) /DT_CONTROL;
    pos_PID = .3*(Kp_pos * error_pos + Ki_pos * integral_pos + Kd_pos * derivative_pos); //scaled by .01 to bring within -1 to 1 range
    error_pos_prev = error_pos;
    targetSpeed = pos_PID;
    targetSpeed = constrainVal(targetSpeed,-0.32,0.32);
  {
    if(angleIMU>180)angleIMU-=360;
    if(angleIMU<-180)angleIMU+=360;
      // Debug.print(robotRealPosition);
      // Debug.print(",");
      // Debug.print(angleIMU);
      // Debug.print(",");
      // Debug.print(targetSpeedRotation);
      // Debug.print(",");
      // Debug.print(targetSpeed);
      // Debug.print("\r\n");
    }
    isActive = (digitalRead(EMG_STOP));
  if (isActive ==0) {   //check status
      integral_yaw = 0;
      integral_pos = 0;
      setMotorLeft(0);
      setMotorRight(0);
      error_yaw_prev = 0;
      error_pos=0;
      return;
    }
    // if(targetSpeedRotation>0.05)
    
    // Debug.println("motor update");
    
    //
    // if(speedLeft>0)speedLeftFeedback = (float)speed_pulse_counter1/DT_CONTROL;
    // else speedLeftFeedback = -(float)speed_pulse_counter1/DT_CONTROL;
    
    // speed_pulse_counter1=0;
    // if(speedRight>0)speedRightFeedback = (float)speed_pulse_counter2/DT_CONTROL;
    // else speedRightFeedback = -(float)speed_pulse_counter2/DT_CONTROL;
    // speed_pulse_counter2=0;
    // robotPosition+=(speedLeftFeedback+speedRightFeedback)/2.0*DT_CONTROL*3.0/11.5;//mm output


    // update speedRobot
    float acc = targetSpeed-speedRobot;
    acc = constrainVal(acc,-ACC_MAX,ACC_MAX);
    speedRobot+=acc;
    // update speedRight speedLeft
    acc = speedRobot-targetSpeedRotation*BASE_LEN/2.0-speedRight;
    acc = constrainVal(acc,-ACC_MAX*2,ACC_MAX*2);
    speedRight+=acc;
    acc= speedRobot+targetSpeedRotation*BASE_LEN/2.0-speedLeft;
    acc = constrainVal(acc,-ACC_MAX*2,ACC_MAX*2);
    speedLeft+=acc;
    
    setMotorLeft(speedLeft);
    setMotorRight(speedRight);
    
    
}
void motorBLVPWM::SetControlValue(float desAngle,float desPos)
{
    //  targetSpeed = speed;//-rotationSpeed*BASE_LEN/2.0;
     yaw_des=desAngle;
     pos_des = desPos;
    //  targetSpeedRotation = -rotationSpeed;// speed+rotationSpeed*BASE_LEN/2.0;
      // Debug.print(angleIMU);
      // Debug.print(",");
    
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
    digitalWrite(M1_FWD,LOW);// stop 
    digitalWrite(M1_REV,LOW);// stop 
    digitalWrite(M1_STOP_MODE,HIGH);//Deceleration stop 
    digitalWrite(M1_MB_FREE,HIGH);//electromagnetic brake released
    digitalWrite(M1_M0,HIGH);//electromagnetic brake released
    analogWrite(M1_PWM,256);
    // attachInterrupt(digitalPinToInterrupt(M1_IN_SPEED), readSpeed1, RISING);
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
    // attachInterrupt(digitalPinToInterrupt(M2_IN_SPEED), readSpeed2, RISING);
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