
class CamData {
public:
  CamData() {}
  float getTagBearing() {
    return angle;
  }
  float x, y;
  float angle;
  int tagid;
  int connectCount;
};




class SenBusDriver {
public:
  SenBusDriver() {
    sensBusBuffi = 0;
  }
  CamData camh7data;
  void Input(unsigned char inputByte) {
    if (h7ConnectCount > 0) h7ConnectCount--;
    camh7data.connectCount = h7ConnectCount;
    sensBusBuff[sensBusBuffi] = inputByte;
    if (sensBusBuffo == 0xAA)
      if (inputByte == 0x55) {
        // DEBUG_TELEMETRY.println(sensBusBuffi);
        if (sensBusBuffi == 12)
          if (sensBusBuff[0] == 0x01) {
            if (sensBusBuff[1] == 0x11) {
              int angleTag = sensBusBuff[7] * 256 + sensBusBuff[8];
              camh7data.x = sensBusBuff[5] - 127;
              camh7data.y = 137 - sensBusBuff[6];
              camh7data.angle = angleTag / 10.0;
              h7ConnectCount = 200;
              // DEBUG_TELEMETRY.print(sensBusBuff[5]);DEBUG_TELEMETRY.print(" ");
              // DEBUG_TELEMETRY.print(sensBusBuff[6]);DEBUG_TELEMETRY.print(" ");
              // DEBUG_TELEMETRY.println(angleTag); DEBUG_TELEMETRY.print(" ");
              // DEBUG_TELEMETRY.print(sensBusBuff[6]);DEBUG_TELEMETRY.print("\r\n");
            }
          }
        sensBusBuffi = 0;
      }
    sensBusBuffo = inputByte;
    sensBusBuffi++;
    if (sensBusBuffi >= 200) sensBusBuffi = 0;
  }
private:
  unsigned char sensBusBuff[100];
  unsigned char sensBusBuffo;
  int h7ConnectCount = 0;
  int sensBusBuffi = 0;

};