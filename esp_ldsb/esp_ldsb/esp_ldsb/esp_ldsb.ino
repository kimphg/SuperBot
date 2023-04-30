#define RXD2 17
#define TXD2 16
int led1=21,led2=27;
void processFrameHex(unsigned char* data)
{
    float angle = data[0]-160;
    int ministep=16;
    // if(angle<0||angle>90)return;
    Serial.println(angle);
    for(int miniangle = 0;miniangle<16;miniangle++)
    {
        int realAngle = angle*ministep+miniangle;
        float range  = data[3+miniangle*2] +data[4+miniangle*2]*256 ;
        // mapData[realAngle] = range;
    }
    // frameCount++;
}
void setup()
{
  //460800
  Serial2.begin(460800, SERIAL_8N1, RXD2, TXD2);
  Serial.begin(460800);
  
  Serial2.print("startlds$");
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  digitalWrite(led1,HIGH);
  digitalWrite(led2,HIGH);
  delay(500);
}

unsigned char dataBuff[200];
int curBuffIndex = 0;
int fragmentCount = 0;
int zeroCount = 0;
void loop()
{
  
  // Serial.println("khang");
  Serial2.print("startlds$");
      digitalWrite(led1,LOW);
      digitalWrite(led2,LOW);
  while (Serial2.available())
  {
    unsigned char bytein = Serial2.read();
    dataBuff[curBuffIndex] = bytein;

    if (bytein)zeroCount = 0;
    else zeroCount++;

    if (zeroCount >= 4)
    {
      // digitalWrite(led1,HIGH);
      // digitalWrite(led2,HIGH);
      // Serial.write(0xff);
      // Serial.write(0xaa);
      if(dataBuff[3]>=160)      processFrameHex(dataBuff + 3);
      curBuffIndex = 0;
      zeroCount = 0; 
      
    }
    else
    {
      curBuffIndex++;
      if (curBuffIndex >= 200)
      {
        curBuffIndex = 0;
      }
    }
  }
}
