
void setup() {
  Serial.begin(460800);
  Serial1.begin(460800);
  delay(200);
  Serial1.print("startlds$");
}


unsigned char dataBuff[200];
int curBuffIndex = 0;
int fragmentCount = 0;
int zeroCount = 0;
void loop() {
  while (Serial1.available()) {
    unsigned char bytein = Serial1.read();
    dataBuff[curBuffIndex] = bytein;
    if(bytein)zeroCount = 0;else zeroCount++;
    if (bytein>=4)
    {
      Serial.w
      

      curBuffIndex = 0;
    }
    curBuffIndex++;
    if (curBuffIndex >= 200)
    {
      curBuffIndex = 0;
    }

  }
}
