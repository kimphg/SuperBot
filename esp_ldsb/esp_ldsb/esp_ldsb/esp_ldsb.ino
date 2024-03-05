#define RXD2 17
#define TXD2 16
int led1 = 21, led2 = 27;
int obstacleData[18];
bool isFrontView = false;
void processFrameHex(unsigned char* data) {
  // float angle = data[0] - 160;
  // int ministep = 16;
  // // if(angle<0||angle>90)return;
  // Serial.println(angle);
  // for (int miniangle = 0; miniangle < 16; miniangle++) {
  //   int intAngle = angle * ministep + miniangle;
  //   float range = data[3 + miniangle * 2] + data[4 + miniangle * 2] * 256;
  //   mapData[intAngle] = range;
  //   float realAz = -intAngle * 3.1415926535 * 2.0 / (90.0 * ministep);
  //   if (realAz < 180) {
  //     isFrontView =true;
  //     int obstacleAngle = realAz / 10;
  //     if (obstacleData[obstacleAngle] < range) obstacleData[obstacleAngle] < range;
  //   }
  //   else 
  //   {
  //     if(isFrontView)
  //     {
  //       isFrontView = false;
  //       //
  //     }

  //   }
  // }
  // frameCount++;
}
const int trig4 = 18;     // chân trig của HC-SR04
const int echo4 = 19;     // chân echo của HC-SR04
 
void readSR04()
{
  unsigned long duration; // biến đo thời gian
    int distance;           // biến lưu khoảng cách
    
    /* Phát xung từ chân trig */
    digitalWrite(trig4,0);   // tắt chân trig
    delayMicroseconds(2);
    digitalWrite(trig4,1);   // phát xung từ chân trig
    delayMicroseconds(5);   // xung có độ dài 5 microSeconds
    digitalWrite(trig4,0);   // tắt chân trig
    
    /* Tính toán thời gian */
    // Đo độ rộng xung HIGH ở chân echo. 
    duration = pulseIn(echo4,HIGH);  
    // Tính khoảng cách đến vật.
    distance = int(duration/2/29.412);
    
    /* In kết quả ra Serial Monitor */
    Serial.print(distance);
    Serial.println("cm");
    delay(200);
}

void setup() {
  //460800
  Serial2.begin(460800, SERIAL_8N1, RXD2, TXD2);
  Serial.begin(460800);

  Serial2.print("startlds$");
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  digitalWrite(led1, HIGH);
  digitalWrite(led2, HIGH);
  delay(500);
      pinMode(trig4,OUTPUT);   // chân trig sẽ phát tín hiệu
    pinMode(echo4,INPUT);    // chân echo sẽ nhận tín hiệu
}

unsigned char dataBuff[200];
int curBuffIndex = 0;
int fragmentCount = 0;
int zeroCount = 0;
void loop() {
  readSR04();
  // Serial2.print("startlds$");
  // digitalWrite(led1, LOW);
  // digitalWrite(led2, LOW);
  // while (Serial2.available()) {
  //   unsigned char bytein = Serial2.read();
  //   dataBuff[curBuffIndex] = bytein;

  //   if (bytein) zeroCount = 0;
  //   else zeroCount++;

  //   if (zeroCount >= 4) {
  //     // digitalWrite(led1,HIGH);
  //     // digitalWrite(led2,HIGH);
  //     // Serial.write(0xff);
  //     // Serial.write(0xaa);
  //     if (dataBuff[3] >= 160) processFrameHex(dataBuff + 3);
  //     curBuffIndex = 0;
  //     zeroCount = 0;

  //   } else {
  //     curBuffIndex++;
  //     if (curBuffIndex >= 200) {
  //       curBuffIndex = 0;
  //     }
  //   }
  // }
}
