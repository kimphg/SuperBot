
void setup() {
  Serial.begin(115200);
  Serial3.begin(460800);
  delay(200);
  Serial3.print("startlds$");
  Serial3.print("startlds$");
}


unsigned char dataBuff[200];
int curBuffIndex = 0;
int fragmentCount = 0;
int zeroCount = 0;
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
