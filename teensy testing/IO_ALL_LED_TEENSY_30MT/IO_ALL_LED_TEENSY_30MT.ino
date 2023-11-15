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
long int time_ms;
long old_dt=4000;
float speedIn2=0.0;
float speedIn1=0.0;
float speedOut2=0.0;
float speedOut1=0.0;
void readSpeed2();
void motorInit2()
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
void motorInit1()
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
void setMotorSpeed1(float speed)
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
void setMotorSpeed2(float speed)
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

void reportData()
{
  Serial.print(speedOut2);
  Serial.print(",");
  Serial.println(speedIn2/1200.0);  
  if((micros() - time_ms)>10000)speedIn2=0;
}

IntervalTimer myTimer;
// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
   motorInit2();
   motorInit1();
        setMotorSpeed2(0.3);  
        myTimer.begin(reportData, 15000); 
        analogWriteFrequency(M2_PWM, 500);
        analogWriteFrequency(M1_PWM, 500);
}

void readSpeed2()
{
  long int  dt_ppm;
  int trig = digitalRead(M2_IN_SPEED);
  if (trig==1) { //only care about rising edge
      dt_ppm = micros() - time_ms;
      if(dt_ppm<100)return;
      time_ms = micros();
      
      old_dt+=(dt_ppm-old_dt);
      speedIn2=(1000000.0/old_dt)*2.0;
      
    }

}
void readSpeed1()
{
  long int  dt_ppm;
  int trig = digitalRead(M2_IN_SPEED);
  if (trig==1) { //only care about rising edge
      dt_ppm = micros() - time_ms;
      if(dt_ppm<100)return;
      time_ms = micros();
      
      old_dt+=(dt_ppm-old_dt);
      speedIn2=(1000000.0/old_dt)*2.0;
      
    }

}
// the loop function runs over and over again forever
void loop() {
  
  // digitalWrite(LED_BUILTIN,HIGH);
  // int m2in = digitalRead(M2_IN_SPEED);
  // Serial.println(m2in);
  speedOut2=0.3;
  
  for(speedOut2=0;speedOut2<=1.0;speedOut2+=0.01)
  {
    delay(20);
  setMotorSpeed2(speedOut2);
   setMotorSpeed1(speedOut2);
  }
  

  delay(2000);
  
  for(speedOut2=1;speedOut2>=0.0;speedOut2-=0.01)
  {
    delay(20);
    setMotorSpeed2(speedOut2);
    setMotorSpeed1(speedOut2);
  }
  delay(2000);
}
