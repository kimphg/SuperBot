
void setup() {
  Serial.begin(115200);
  Serial3.begin(460800);
  delay(200);
  Serial3.print("startlds$");
  Serial3.print("startlds$");
}

#define MAX_AZ 1440
float mapData[MAX_AZ];
unsigned int mapTime[MAX_AZ];
unsigned char dataBuff[200];
int curBuffIndex = 0;
int fragmentCount = 0;
int zeroCount = 0;

void processFrameHex(unsigned char* data)
{
    float angle = data[0]-160;
    if(angle<0||angle>90)return;
    for(int miniangle = 0;miniangle<16;miniangle++)
    {
        int realAngle = angle*16+miniangle;
        
        if((realAngle>= 1380)||(realAngle<=540))
        {
          // Serial.println(realAngle);
          float newrange  = data[3+miniangle*2] +data[4+miniangle*2]*256 ;
          float oldrange = mapData[realAngle];
          mapData[realAngle] = newrange;

          float realAzDeg = -60+realAngle*180*2.0/(1440);
          if(realAzDeg<-180)realAzDeg+=(360);
          if(realAzDeg>180)realAzDeg-=(360);
          
          
          if (range<1){ continue;}
          float xmm = range*sin(realAzDeg/57.2957795);
          float ymm = range*cos(realAzDeg/57.2957795);
          int safe_level = 0;
          if(abs(xmm)<400)
            {
                if(ymm<300)safe_level=0;
                else if(ymm<1500)safe_level=1;
                else safe_level=2;
            }
            else if(abs(xmm)<800)
            {
                if(ymm<200)safe_level=0;
                else if(ymm<600)safe_level=1;
                else safe_level=2;

            }
            else safe_level=2;
            Serial.print(realAzDeg);
          Serial.print(',');
          Serial.print(newrange);
          Serial.print(',');
          Serial.print(safe_level);
          Serial.println(',');
        }
    }

}
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
