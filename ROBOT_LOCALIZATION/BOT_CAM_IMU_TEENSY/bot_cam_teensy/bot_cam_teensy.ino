#include <SPI.h>  // include the SPI library:
SPISettings settingsA(5000000, MSBFIRST, SPI_MODE0);
IntervalTimer myTimer;
const int slaveSelectPin = 10;
void setup() {
  // set the slaveSelectPin as an output:
  pinMode(slaveSelectPin, OUTPUT);
  // initialize SPI:
  SPI.begin();
  myTimer.begin(IMU_CSR_2100_READ, 750);
  Serial1.begin(1000000);
}
uint8_t CalculateCRC(uint32_t Data) {
  uint8_t BitIndex;
  uint8_t BitValue;
  uint8_t CRC;
  CRC = 0xFF;
  for (BitIndex = 31; BitIndex > 7; BitIndex--) {
    BitValue = (uint8_t)((Data >> BitIndex) & 0x01);
    CRC = CRC8(BitValue, CRC);
  }
  CRC = (uint8_t)~CRC;
  return CRC;
}
static uint8_t CRC8(uint8_t BitValue, uint8_t CRC) {
  uint8_t Temp;
  Temp = (uint8_t)(CRC & 0x80);
  if (BitValue == 0x01) {
    Temp ^= 0x80;
  }
  CRC <<= 1;
  if (Temp > 0) {
    CRC ^= 0x1D;
  }
  return CRC;
}
int32_t dataIMU[256];
int count=0;
float gyroValue=0;
float angleValue=0;
void IMU_CSR_2100_READ() {
  count++;
  int start=0, end=3;
  for (int addr = start; addr < end; addr++) {
    SPI.beginTransaction(settingsA);
    digitalWrite(slaveSelectPin, LOW);
    byte readAddr = addr << 2;
    SPI.transfer(readAddr);
    dataIMU[addr] = SPI.transfer(0) << 8;
    dataIMU[addr] |= SPI.transfer(0);
    if (dataIMU[addr] > 32767) dataIMU[addr] -= 65536;
    int cs = SPI.transfer(CalculateCRC((uint32_t)readAddr << 24));
    digitalWrite(slaveSelectPin, HIGH);
    SPI.endTransaction();
  }
  gyroValue = dataIMU[2]/61000.0;
  angleValue+= gyroValue;
}

int sec=0;
void loop() {
  if(int(millis()/1000)!=sec)
  {
    sec = int(millis()/10);
    // Serial.print(angleValue);
    // Serial.print(" ");
    // Serial.println(count);
    count=0;
  }
  if(Serial1.available())
  {
    Serial.println(Serial1.read());

  }
  
}