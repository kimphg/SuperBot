
#include "motorBLVPWM.h"
float constrainVal(float input,float min, float max)
{
  if(input<min)return min;
  if(input>max)return max;
  return input;
}
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
    robotPosition=0;
    i_limit_yaw = 3.0;
    // i_limit_pos = 20;
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
    robotPosition=0;
    Kp_yaw = 0.4;          //Yaw P-gain
    Ki_yaw = 0.1;          //Yaw I-gain
    Kd_yaw = 0.01;         //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
    yaw_des= 0;
    Kp_pos = 0.01;          //Yaw P-gain
    Ki_pos = 0.0;          //Yaw I-gain
    Kd_pos = 0.00;         //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
    pos_des= 0;
}


void motorBLVPWM::update(float angleIMU)
{
    {
      Serial.print(robotPosition);
      Serial.print(",");
      Serial.print(pos_des);
      Serial.print(",");
      Serial.print(yaw_des);
      Serial.print(",");
      Serial.print(angleIMU);
      Serial.print("\r\n");
    }
  if (isActive ==0) {   //check status
      integral_yaw = 0;
      integral_pos = 0;
      setMotorLeft(0);
      setMotorRight(0);
      error_yaw_prev = 0;
      error_pos=0;
      return;
    }
    unsigned long int newTime = millis();
    int dt = newTime-timeMillis;//check dt, should be 20ms
    timeMillis=newTime;
    // Serial.println(dt);
    if(dt<1)return;//dt too small

    error_yaw = yaw_des - angleIMU;
    if(error_yaw>180)error_yaw-=360;
    if(error_yaw<-180)error_yaw+=360;
    integral_yaw +=  error_yaw * DT_CONTROL;
    integral_yaw = constrainVal(integral_yaw, -i_limit_yaw, i_limit_yaw); //saturate integrator to prevent unsafe buildup
    derivative_yaw = (error_yaw - error_yaw_prev) /DT_CONTROL;
    yaw_PID = .3 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw); //scaled by .01 to bring within -1 to 1 range
    error_yaw_prev = error_yaw;

    targetSpeedRotation = -yaw_PID;
    targetSpeedRotation = constrainVal(targetSpeedRotation,-1,1);

    error_pos = pos_des - robotPosition;
    integral_pos +=  error_pos * DT_CONTROL;
    integral_pos = constrainVal(integral_pos, -i_limit_pos, i_limit_pos); //saturate integrator to prevent unsafe buildup
    derivative_pos = (error_pos - error_pos_prev) /DT_CONTROL;
    pos_PID = .3*(Kp_pos * error_pos + Ki_pos * integral_pos + Kd_pos * derivative_pos); //scaled by .01 to bring within -1 to 1 range
    error_pos_prev = error_pos;
    targetSpeed = pos_PID;
    targetSpeed = constrainVal(targetSpeed,-0.3,0.3);

    // if(targetSpeedRotation>0.05)
    
    // Serial.println("motor update");
    
    //
    if(speedLeft>0)speedLeftFeedback = (float)speed_pulse_counter1/DT_CONTROL;
    else speedLeftFeedback = -(float)speed_pulse_counter1/DT_CONTROL;
    
    speed_pulse_counter1=0;
    if(speedRight>0)speedRightFeedback = (float)speed_pulse_counter2/DT_CONTROL;
    else speedRightFeedback = -(float)speed_pulse_counter2/DT_CONTROL;
    speed_pulse_counter2=0;
    robotPosition+=(speedLeftFeedback+speedRightFeedback)/2.0*DT_CONTROL*3.0/11.5;//mm output


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
      // Serial.print(angleIMU);
      // Serial.print(",");
    
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