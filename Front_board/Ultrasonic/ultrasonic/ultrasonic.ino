const int trig = 2;
const int echo1 = A0;
const int echo2 = A1;
const int echo3 = A2;
const int echo4 = A3;
const int buzzer  = 7;
void setup()
{
  Serial.begin(115200);
  pinMode(trig, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(echo2, INPUT);
  pinMode(echo3, INPUT);
  pinMode(echo4, INPUT);
  digitalWrite(buzzer, LOW);  
  delay(100);
  digitalWrite(buzzer, HIGH);  
  delay(100);
  digitalWrite(buzzer, LOW);  
  delay(300);
  digitalWrite(buzzer, HIGH);  
  delay(100);
  digitalWrite(buzzer, LOW);  
  delay(100);

}
void loop(){
  digitalWrite(trig, LOW);  
	delayMicroseconds(2);  
	digitalWrite(trig, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(trig, LOW);
  uint32_t startTime = micros();
  if(startTime>4294937295)return;//avoid reset moment

  uint32_t startTime1=0,startTime2=0,startTime3=0,startTime4=0 ;
  // delayMicroseconds(20);
  uint32_t echotime1=0,echotime2=0,echotime3=0,echotime4=0;
  while (1){
    uint32_t curtime = micros();
    
  if(startTime1==0&&(digitalRead(echo1)==HIGH))startTime1 = curtime;//record the pulse start
  if(startTime2==0&&(digitalRead(echo2)==HIGH))startTime2 = curtime;//record the pulse start
  if(startTime3==0&&(digitalRead(echo3)==HIGH))startTime3 = curtime;//record the pulse start
  if(startTime4==0&&(digitalRead(echo4)==HIGH))startTime4 = curtime;//record the pulse start
  if((startTime1)&&(digitalRead(echo1)==LOW)&&(echotime1==0))echotime1 = curtime-startTime1;//record the pulse stop
  if((startTime2)&&(digitalRead(echo2)==LOW)&&(echotime2==0))echotime2 = curtime-startTime2;//record the pulse stop
  if((startTime3)&&(digitalRead(echo3)==LOW)&&(echotime3==0))echotime3 = curtime-startTime3;//record the pulse stop
  if((startTime4)&&(digitalRead(echo4)==LOW)&&(echotime4==0))echotime4 = curtime-startTime4;//record the pulse stop
    if(curtime-startTime>20000) break;
    if((echotime1*echotime2*echotime3*echotime4)!=0)break;
  }
  //  echotime1 = pulseIn(A0, LOW, 5000);
  //  for(int i=0;i<50;i++){
  //  Serial.println(digitalRead(A0));
  //  delayMicroseconds(20);
  //  }Serial.flush();
  //  delay(100);return;
  // while ((echotime1*echotime2*echotime3*echotime4)==0)
  // {
  //   if(echotime1==0)
  //   if(digitalRead(A0)==0)
  //   {
  //     echotime1 = micros()-startTime;
  //   }
  //   if(echotime2==0)
  //   if(digitalRead(A1)==0)
  //   {
  //     echotime2 = micros()-startTime;
  //   }
  //   if(echotime3==0)
  //   if(digitalRead(A2)==0)
  //   {
  //     echotime3 = micros()-startTime;
  //   }
  //   if(echotime4==0)
  //   if(digitalRead(A3)==0)
  //   {
  //     echotime4 = micros()-startTime;
  //   }
  // }
  float distance1 = echotime1*0.17;
  float distance2 = echotime2*0.17;
  float distance3 = echotime3*0.17;
  float distance4 = echotime4*0.17;
  Serial.print("$SSON,");
  float minDistance = 5000;
  if(minDistance>distance1)minDistance=distance1;
  if(minDistance>distance2)minDistance=distance2;
  if(minDistance>distance3)minDistance=distance3;
  if(minDistance>distance4)minDistance=distance4;
  Serial.print(distance1);  Serial.print(',');
  Serial.print(distance2);  Serial.print(',');
  Serial.print(distance3);  Serial.print(',');
  Serial.print(distance4);  Serial.print(',');
  Serial.print(minDistance);  Serial.print(',');
  Serial.print('\n');
  Serial.flush();
  if(minDistance<500)tone(buzzer,10000-minDistance*10);
  else noTone(buzzer);
  delay(50);
}