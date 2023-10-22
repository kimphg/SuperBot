#ifndef MTI_H
#define MTI_H
#define BUF_SIZE_IMU 100
bool compareArray(unsigned char *data1, unsigned char *data2, int len) {
  for (int i = 0; i < len; i++) {
    if (data1[i] != data2[i]) {
      Serial.print(data1[i], HEX);
      Serial.print(data2[i], HEX);
      return false;
    }
  }
  return true;
}
void printArray(unsigned char *data1, int len) {
  for (int i = 0; i < len; i++) {
    Serial.print(" 0x");
    Serial.print(data1[i], HEX);
  }
  Serial.print("\n");
  return;
}
typedef struct
{

} IMUData;
class IMU_driver {
public:
  IMU_driver() {}
  void IMU_init(Stream &porti) {
    port = &porti;
    port->setTimeout(100);
    isConnected = false;
    Connect();
    gotoMeasurement();
  }
  bool Connect() {
    Serial.println("Connect IMU");
    return gotoConfig();
  }

  bool getIsConnected() {
    return isConnected;
  }
  bool gotoConfig() {
    Serial.println("gotoConfig");

    int trycount = 0;
    bool isSuccess = false;

    unsigned char req[] = { 0xFA, 0xFF, 0x30, 0x00, 0xD1 };
    unsigned char ansExpected[] = { 0xFA, 0xFF, 0x31, 0x00, 0xD0 };
    unsigned char ansBuff[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    while ((trycount < 5)) {
      trycount++;
      delay(50);
      while (port->available()) { port->read(); }  // clear input of serial port

      port->write((char *)req, 5);
      port->flush();

      delay(50);
      if (port->available() >= 5) {
        port->readBytes(ansBuff, 5);
        printArray(ansBuff, 5);
      } else {
        Serial.print("no ans:");
        Serial.println(port->available());
        continue;
      }
      isSuccess = compareArray(ansExpected, ansBuff, 5);
      Serial.println(isSuccess);
      if (isSuccess) break;
    }
    isConnected = isSuccess;
    return isSuccess;
  }
  bool gotoMeasurement() {
    Serial.println("gotoMeasurement");
    int trycount = 0;
    bool isSuccess = false;
    unsigned char req[] = { 0xFA, 0xFF, 0x10, 0x00, 0xF1 };
    unsigned char ansExpected[] = { 0xFA, 0xFF, 0x11, 0x00, 0xF0 };
    unsigned char ansBuff[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    while (trycount < 5) {
      trycount++;
      delay(2);
      while (port->available()) { port->read(); }  // clear input of serial port
      port->write((char *)req, 5);
      port->flush();
      delay(5);
      if (port->available() >= 5) {
        // đọc chữ liệu
        port->readBytes(ansBuff, 5);
      }
      isSuccess = compareArray(ansExpected, ansBuff, 5);
      Serial.println(isSuccess);
      if (isSuccess) break;
    }
    if(isSuccess)Serial.println("gotoMeasurement ok");
    else Serial.println("gotoMeasurement failed");
    return isSuccess;
  }
  void updateData() {
    
  
    while (port->available()>0)
    {
      if(buffIndex>=BUF_SIZE_IMU)buffIndex=0;
      int bytein = port->read();
      // Serial.println(bytein,HEX);
      if(bytein==0xFF)
      if(lastbyte==0xFA)
      {
        printArray(databuf,buffIndex+1);
        buffIndex = 0;
        
      }
      databuf[buffIndex]=bytein;
      buffIndex++;
      lastbyte = bytein;
    }
    //to be implemented
  }
  IMUData getMeasurement() {
    return measurement;
  }
private:
  unsigned char databuf[BUF_SIZE_IMU];
  int buffIndex;
  unsigned char lastbyte;
  IMUData measurement;
  Stream *port;
  bool isConnected;
};
#endif