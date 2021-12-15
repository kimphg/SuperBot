#define BUF_SIZE 100
#define F_SIZE 62
unsigned char lidarRawBuff[BUF_SIZE];
int lidarMap[150][50];

void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
//  Serial1.begin(480600);
  Serial2.begin(480600);
  pinMode(13,OUTPUT);
  delay(50);

  Serial2.print("startlds$");
  digitalWrite(13,HIGH);
  delay(1000);
//  Serial2.write(36);
//  memset(lidarMap,0,12750);
}
int k=0;
unsigned char oldByte = 0;
int byteIndex = 0;
int frameCounter = 0;
unsigned char getBufOffset(int indexOffset)
{
  int resIndex = byteIndex + indexOffset;
  while(resIndex<0)resIndex+=BUF_SIZE;
  while(resIndex>=BUF_SIZE)resIndex-=BUF_SIZE;
  return lidarRawBuff[resIndex];
  }
  unsigned char getFrameByte(int index)
{
  int resIndex = byteIndex - F_SIZE + index;
  while(resIndex<0)resIndex+=BUF_SIZE;
  while(resIndex>=BUF_SIZE)resIndex-=BUF_SIZE;
  return lidarRawBuff[resIndex];
  }
//  int totalByte = 0;
int oldAngleByte=-1;
int aziCount=0;
void loop() {
  digitalWrite(13,LOW);
//  if(millis()>10000)return;
//  Serial.println(controlData[8]);
  while(Serial2.available()) {
    
    unsigned char inByte = Serial2.read();
 
    byteIndex++;
    if(byteIndex>=BUF_SIZE)byteIndex=0;
    lidarRawBuff[byteIndex]=inByte;
//    Serial.print(inByte);
//    Serial.print(',');
    if(inByte==250)
    {
      if(getBufOffset(-6)==0)
      if(getBufOffset(-5)==0)
      if(getBufOffset(-4)==0)
      if(getBufOffset(-3)==0)
      if(getBufOffset(-22)==0)
      if(getBufOffset(-21)==0)//new frame detected
      {
          int angle = getFrameByte(39);
          aziCount++;
          if(aziCount>120)aziCount=120;
          if(angle<oldAngleByte)
          {
//            Serial.print('\n');
//            Serial.print(aziCount);
            aziCount=0;
            digitalWrite(13,HIGH);
            }
          oldAngleByte = angle;
          int rmax = 0;
          int imax = 0;
//          Serial.print('\n');
          Serial.print('$');
          Serial.print(' ');
          Serial.print(getFrameByte(1)-160);
          Serial.print(' ');
          for(int i=0;i<16;i++)
          {
            lidarMap[aziCount][i] = getFrameByte(i*2+5)*256+getBufOffset(i*2+6);
            Serial.print(lidarMap[aziCount][i]);
//              Serial.print(getFrameByte(i*2+5));
//              Serial.print(" ");
//              Serial.print(getFrameByte(i*2+6));
            Serial.print(' ');
//            if(lidarMap[aziCount][i]>rmax)
//            {
//              rmax = lidarMap[aziCount][i];
//              imax = i;
//              }
          }
//          Serial.print("| |");
//          Serial.print(imax);
//          Serial.print(" ");
//          int angle = getBufOffset(-57+18*2);
//          int counter = getFrameByte(1);
//          int angle = getFrameByte(39);
//          Serial.print(counter);
//          Serial.print(" ");
//          Serial.print(angle);
//          Serial.print(lidarMap[16]);
//          totalByte=0;
//          Serial.print('\n');
      }
      
     }
    
//    if(millis()>20000)
//    {
//      for(int angle=0;angle<100;angle++)
//      {
//        Serial.print(angle);
//        Serial.print(' ');
//        for(int i=0;i<16;i++)
//          {
//            Serial.print( lidarMap[angle][i]/1000.0);
//            Serial.print(' ');
//          }
//          Serial.print('\n');
//        }
//        while(1);
//      }
//    
  } 



  // read from port 0, send to port 1:
//  if (Serial2.available()) {
//    int inByte = Serial2.read();
//    Serial.print("data2:");
//    Serial.println(inByte);
//  }
}
