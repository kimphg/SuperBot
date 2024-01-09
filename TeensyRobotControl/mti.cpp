#include "mti.h"
unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
//INTERRUPT SERVICE ROUTINES (for reading PWM and PPM)

float bytesToFloat(unsigned char  b0, unsigned char  b1, unsigned char b2, unsigned char  b3)
{
  float output;

  *((unsigned char*)(&output) + 3) = b0;
  *((unsigned char*)(&output) + 2) = b1;
  *((unsigned char*)(&output) + 1) = b2;
  *((unsigned char*)(&output) + 0) = b3;

  return output;
}

void printArray(unsigned char *data1, int len) {
  for (int i = 0; i < len; i++) {
    Serial.print(" 0x");
    Serial.print(data1[i], HEX);
  }
  Serial.print("\n");
  return;
}
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
IMU_driver::IMU_driver()
{
    measurement.gyroZold=0;
    yawShift = 0;
    yawCalcMode = 0;
}

void IMU_driver::IMU_init(Stream &porti)
{
    port = &porti;
    // port->setTimeout(100);
    isConnected = false;
      gyroZBias =0;
      gyroZBiasCompensation=0;
   gyroZBiasCount=0;
   measurement.gyroyaw = 0;
    Connect();
    gotoMeasurement();
}

void IMU_driver::resetYaw()
{
  measurement.gyroyaw = 0;
  yawShift = measurement.gyroyaw - measurement.yaw;
  while(yawShift>360.0)yawShift-=360.0;
                while(yawShift<0)yawShift+=360.0;
}

bool IMU_driver::Connect()
{
    Serial.println("Connect IMU");
    return gotoConfig();
}

bool IMU_driver::gotoConfig()
{
    Serial.println("gotoConfig");

    int trycount = 0;
    bool isSuccess = false;

    unsigned char req[] = { 0xFA, 0xFF, 0x30, 0x00, 0xD1 };
    unsigned char ansExpected[] = { 0xFA, 0xFF, 0x31, 0x00, 0xD0 };
    unsigned char ansBuff[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    while (trycount < 5){
      trycount++;
      delay(20);
      while (port->available()) {
        port->read();
      }  // clear input of serial port

      port->write((char *)req, 5);
      port->flush();

      delay(20);
      if (port->available() >= 5) {
        port->readBytes(ansBuff, 5);
      } else {
        Serial.print("no ans:");
        Serial.println(port->available());
        continue;
      }
      isSuccess = compareArray(ansExpected, ansBuff, 5);
      if (isSuccess) {
        Serial.print(trycount);
        Serial.println(" try counts, gotoConfig ok");
        isMeasurement = false;
        break;
      }
    }
    isConnected = isSuccess;
    if (!isSuccess) Serial.println("gotoConfig failed");
    return isSuccess;
  }
  void IMU_driver::updateData()
 {
    while (port->available() > 0) {
      if (buffIndex >= BUF_SIZE_IMU) buffIndex = 0;
      uint8_t bytein = port->read();
//      Serial.write(&bytein,1);
      if (bytein == 0xFF)
        if (lastbyte == 0xFA) {
          // printArray(databuf, buffIndex + 1);
          if (buffIndex > 15) {
            measurement.mid = databuf[1];
            int dataLen = databuf[2];
            int iti = 3;
            while(iti<dataLen)
            {
              
              int xdi = (databuf[iti] << 8) | databuf[iti+1];
              // Serial.println(xdi);
              unsigned char leni = databuf[iti+2];
              if (xdi == 8240) {  //MTDATA2 data ID of euler angles
                measurement.roll =  bytesToFloat(databuf[iti+3], databuf[iti+4], databuf[iti+5], databuf[iti+6]);
                measurement.pitch = bytesToFloat(databuf[iti+7], databuf[iti+8], databuf[iti+9], databuf[iti+10]);
                measurement.yaw =   bytesToFloat(databuf[iti+11], databuf[iti+12], databuf[iti+13], databuf[iti+17]);
                if(yawCalcMode==0){
                  yawShift = measurement.gyroyaw-measurement.yaw;
                  while(yawShift>360.0)yawShift-=360.0;
                  while(yawShift<0)yawShift+=360.0;
                }
                else
                {
                  measurement.gyroyaw = measurement.yaw+yawShift;
                  while(measurement.gyroyaw>360.0)measurement.gyroyaw-=360.0;
                  while(measurement.gyroyaw<0)measurement.gyroyaw+=360.0;
                }
                // Serial.println(measurement.yaw);
                // Serial.println("new euler angles");
              }
              if (xdi == 32832) {  //MTDATA2 data ID of rate of turn HR
                measurement.gyroX =  bytesToFloat(databuf[iti+3], databuf[iti+4], databuf[iti+5], databuf[iti+6]);
                measurement.gyroY = bytesToFloat(databuf[iti+7], databuf[iti+8], databuf[iti+9], databuf[iti+10]);
                float newGyroZ =   -bytesToFloat(databuf[iti+11], databuf[iti+12], databuf[iti+13], databuf[iti+17]);
                if(abs(newGyroZ<0.1))
                {
                  if(yawCalcMode>0)
                  yawCalcMode--;
                }
                else yawCalcMode=200;
                if(yawCalcMode==0){
                  if(abs(newGyroZ)>0.01)
                  {
                    // Serial.println(newGyroZ-measurement.gyroZ);
                    noMotionCount=0;
                  }
                  if(noMotionCount>100)
                  {
                    gyroZBias+=newGyroZ;
                    gyroZBiasCount++;
                    if(gyroZBiasCount>200)
                    {
                      float newBias = gyroZBias/gyroZBiasCount;
                      gyroZBiasCount=0;
                      gyroZBias=0;
                      gyroZBiasCompensation+=0.4*(newBias-gyroZBiasCompensation);
                      // Serial.println(1000*gyroZBiasCompensation);
                    }
                  }
                  
                  measurement.gyroZ=newGyroZ-gyroZBiasCompensation;
                  
                  measurement.gyroyaw+= (measurement.gyroZ+measurement.gyroZold)/1000.0*57.2958;// todo: add dt later
                  
                  while(measurement.gyroyaw>360.0)measurement.gyroyaw-=360.0;
                  while(measurement.gyroyaw<0)measurement.gyroyaw+=360.0;
                }
                measurement.gyroZold = measurement.gyroZ;
                // Serial.print(measurement.gyroZ);
                // Serial.print(" ");
                dataUpdated =true;
                // Serial.println("new gyro data");
              }
              if (xdi == 16448) {  //MTDATA2 data ID of acceleration HR
                measurement.accX =  bytesToFloat(databuf[iti+3], databuf[iti+4], databuf[iti+5], databuf[iti+6]);
                measurement.accY =  bytesToFloat(databuf[iti+7], databuf[iti+8], databuf[iti+9], databuf[iti+10]);
                float newaccZ =   bytesToFloat(databuf[iti+11], databuf[iti+12], databuf[iti+13], databuf[iti+17]);
                // Serial.println(abs(newaccZ-measurement.accZ));
                if(abs(newaccZ-measurement.accZ)>0.3)
                {
                  // Serial.println(noMotionCount);
                  noMotionCount=0;
                }
                else noMotionCount++;
                measurement.accZ = newaccZ;
                
                
                // Serial.println("new acc data");
              }
              iti+=leni+1;
            }
          }
          buffIndex = 0;
        }
      databuf[buffIndex] = bytein;
      buffIndex++;
      lastbyte = bytein;
    }
    //to be implemented
  }
  bool IMU_driver::gotoMeasurement()
  {
    Serial.println("gotoMeasurement");
    int trycount = 0;
    bool isSuccess = false;
    unsigned char req[] = { 0xFA, 0xFF, 0x10, 0x00, 0xF1 };
    unsigned char ansExpected[] = { 0xFA, 0xFF, 0x11, 0x00, 0xF0 };
    unsigned char ansBuff[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    while (trycount < 5) {
      trycount++;
      delay(20);
      while (port->available()) { port->read(); }  // clear input of serial port
      port->write((char *)req, 5);
      port->flush();
      delay(20);
      if (port->available() >= 5) {
        // đọc chữ liệu
        port->readBytes(ansBuff, 5);
      }
      isSuccess = compareArray(ansExpected, ansBuff, 5);
      if (isSuccess) break;
    }
    if (isSuccess) {
      Serial.print(trycount);
      Serial.println(" try counts, gotoMeasurement ok");
    } else Serial.println("gotoMeasurement failed");
    return isSuccess;
  }
