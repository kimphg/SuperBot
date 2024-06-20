
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
    int ministep=16;
    int frameCount =0;
    for(int miniangle = 0;miniangle<16;miniangle++)
    {
        int realAngle = angle*ministep+miniangle;
        float range  = data[3+miniangle*2] +data[4+miniangle*2]*256 ;
        float oldRange = mapData[realAngle];
        float oldTime = mapTime[realAngle];
        mapData[realAngle] = range;
        mapTime[realAngle] = millis();
        
        frameCount++;
        if(frameCount>1500)
        {
            int prevAngle = realAngle-1;
            if(prevAngle<0)prevAngle += MAX_AZ;
            if(abs(mapData[prevAngle]-mapData[realAngle])<5)
            {
              float dRange =  oldRange-range;
              float dTime = mapTime[realAngle]-oldTime;
              float approSpd = dRange/dTime;
            }
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
      Serial.write(0xff);
      Serial.write(0xaa);
      Serial.write(dataBuff+3, 35);
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
