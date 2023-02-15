
void setup() {
  Serial.begin(460800);
  Serial1.begin(460800);
  delay(200);
  Serial1.print("startlds$");
}
long int timelast = 0;
long int timenew = 0;
#define BUFF_LEN 30
unsigned char dataBuff[BUFF_LEN][200];
int curBuffIndex = 0;
int fragmentCount = 0;
void loop() {
  while (Serial1.available()) {
    dataBuff[fragmentCount][curBuffIndex] = Serial1.read();
    timenew = micros();
    // mỗi frame là 62 byte, cứ 3 frame lại có 1 khoảng trống 2ms nên có thể tách thành các fragment, mỗi fragment 3 frame
    if (timenew - timelast > 1000)
    {
      fragmentCount++;
      if (fragmentCount >= BUFF_LEN) {
        for (int fragment = 0; fragment < BUFF_LEN; fragment++)
        {
          for (int part = 0; part < 3; part++)// mỗi part là 1 frame
          {
            unsigned char *frameData = &(dataBuff[fragment][0])+62*part;
            Serial.write(0xff);
            Serial.write(0xAA);
//            Serial.println(frameData[2]);
            Serial.write(&frameData[2],35);//
//            Serial1.print("startlds$");
//            Serial.print(frameData[2] - 160); Serial.print(' ');
//            for (int i = 5; i < 37; i += 2)
//            {
//              int valuerange = frameData[i] + frameData[i + 1] * 256;
//              Serial.print(valuerange); Serial.print(' ');
//            }
//            Serial.print('\n');
          }
        }
        fragmentCount = 0;
      }

      curBuffIndex = 0;
    }
    timelast = timenew;
    curBuffIndex++;
    if (curBuffIndex >= 200)
    {
      curBuffIndex = 0;
    }

  }
}
