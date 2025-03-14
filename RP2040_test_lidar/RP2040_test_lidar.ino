
#define LED_1 23
#define LED_2 22
#define LED_3 4
#define LED_4 5
#define LED_5 9
#define LED_6 10
#define MAX_AZ 1440
float mapData[MAX_AZ];
unsigned int mapTime[MAX_AZ];
unsigned char dataBuff[200];
int curBuffIndex = 0;
int fragmentCount = 0;
int zeroCount = 0;
IntervalTimer myTimer;
bool newFrameAvailable = false;
int warning_level = 1;
void Process200ms()
{
  if(newFrameAvailable)
  {
    Serial1.print("$FRB,");
    Serial1.print(warning_level);
    Serial1.println(",#");
    if(warning_level==0){
      
      // analogWrite(LED_1,220);
      // analogWrite(LED_2,220);
      // analogWrite(LED_3,220);
      // analogWrite(LED_4,220);
      // analogWrite(LED_5,220);
      // analogWrite(LED_6,220);
    }
    if(warning_level==1){
      analogWrite(LED_1,150);
      analogWrite(LED_2,150);
    }
    else{
      analogWrite(LED_1,255);
      analogWrite(LED_2,255);
    }
    if(warning_level==2){
      analogWrite(LED_3,150);
      analogWrite(LED_4,150);
    }
    else{
      analogWrite(LED_3,255);
      analogWrite(LED_4,255);
    }
    if(warning_level==3){
      analogWrite(LED_5,150);
      analogWrite(LED_6,150);
    }
    else{
      analogWrite(LED_5,255);
      analogWrite(LED_6,255);
    }

    newFrameAvailable=false;
    // Serial.println(warning_level);
    warning_level = 0;
  }
  else {
    
    Serial3.print("startlds$");
  }
}
// Modbus MBclient1(1,Serial2,0);
void setup() {
  Serial.begin(115200);
  Serial1.begin(921600);
  Serial3.begin(460800);
  // MBclient1.begin(Serial2,9600);
  delay(200);
  Serial3.print("startlds$");
  Serial3.print("startlds$");
  pinMode(LED_1,OUTPUT);
  pinMode(LED_2,OUTPUT);
  pinMode(LED_3,OUTPUT);
  pinMode(LED_4,OUTPUT);
  pinMode(LED_5,OUTPUT);
  pinMode(LED_6,OUTPUT);
  myTimer.begin(Process200ms, 150000); 
  analogWrite(LED_1,220);
      analogWrite(LED_2,220);
      analogWrite(LED_3,220);
      analogWrite(LED_4,220);
      analogWrite(LED_5,220);
      analogWrite(LED_6,220);
      delay(200);
      analogWrite(LED_1,150);
      analogWrite(LED_2,150);
      analogWrite(LED_3,150);
      analogWrite(LED_4,150);
      analogWrite(LED_5,150);
      analogWrite(LED_6,150);
      delay(200);
      analogWrite(LED_1,50);
      analogWrite(LED_2,50);
      analogWrite(LED_3,50);
      analogWrite(LED_4,50);
      analogWrite(LED_5,50);
      analogWrite(LED_6,50);
      delay(200);
      analogWrite(LED_1,250);
      analogWrite(LED_2,250);
      analogWrite(LED_3,250);
      analogWrite(LED_4,250);
      analogWrite(LED_5,250);
      analogWrite(LED_6,250);
}



void processFrameHex(unsigned char* data)
{
    float angle = data[0]-160;
    if(angle<0||angle>90)return;
    for(int miniangle = 0;miniangle<16;miniangle++)
    {
        int realAngle = angle*16+miniangle;
        if(realAngle==600)newFrameAvailable = true;
        if((realAngle>= 1390)||(realAngle<=530))
        {
          // Serial.println(realAngle);
          float range  = data[3+miniangle*2] +data[4+miniangle*2]*256 ;
          float oldrange = mapData[realAngle];
          
          mapData[realAngle] = range;
          // if(range>oldrange)range=oldrange;
          float realAzDeg = -60+realAngle*180*2.0/(1440);
          if(realAzDeg<-180)realAzDeg+=(360);
          if(realAzDeg>180)realAzDeg-=(360);
          
          
          if (range<10){ continue;}
          float xmm = range*sin(realAzDeg/57.2957795);
          float ymm = range*cos(realAzDeg/57.2957795);
          if(ymm<50)continue;
          int new_warning_level=1;
          if((abs(xmm))<290)
            {
                if(ymm<250)new_warning_level=3;
                else if(ymm<500)new_warning_level=2;
                else new_warning_level=1;
            }
            
            
            Serial.print(realAzDeg);
          Serial.print(',');
          Serial.print(range);
          Serial.print(',');
          Serial.print(new_warning_level);
          Serial.println(',');
          if(new_warning_level>warning_level)warning_level=new_warning_level;
        }
    }

}
int u8state = 0;
void loop() {
  
  while (Serial3.available()) {
    unsigned char bytein = Serial3.read();
    dataBuff[curBuffIndex] = bytein;
    if (bytein)zeroCount = 0; else zeroCount++;
    if (zeroCount >= 4)
    {
      // Serial.write(0xff);
      // Serial.write(0xaa);
      // Serial.write(dataBuff+3, 35);
      processFrameHex(dataBuff+3);
      curBuffIndex = 0;
      zeroCount=0;
    }
    else
    { curBuffIndex++;
      if (curBuffIndex >= 200)
      {
        curBuffIndex = 0;
      }
    }

  }
}
