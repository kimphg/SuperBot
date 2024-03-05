#include <stdint.h>
#ifndef MTI_H
#define MTI_H
#define BUF_SIZE_IMU 100
#include "common.h"
#include <Arduino.h>


typedef struct
{
  int mid;
  int dataLen;
  int xdi;
  float roll, pitch, yaw, gyroyaw;
  float gyroX,gyroY,gyroZ,gyroZold;
  float accX,accY,accZ;

} IMUData;
class IMU_driver {
public:
  float yawShift;
  int yawCalcMode ;
  int failCount = 0;
  IMU_driver();
  void IMU_init(Stream *porti) ;
  void resetYaw(float newyaw);
  bool Connect() ;
  bool gotoMeasurement();
  inline bool isConnected() {
     return conected;
  }
  bool gotoConfig() ;
  void updateData() ;
  
  inline IMUData getMeasurement() {
    return measurement;
  }
  // bool isUpdated;
  int noMotionCount=0;
  float gyroZBiasCompensation;
private:
  // void sendControlPacket();
  float gyroZBias=0;
  int gyroZBiasCount;
  unsigned char databuf[BUF_SIZE_IMU];
  int buffIndex=0;
  
  unsigned char lastbyte;
  IMUData measurement;
  Stream *port;
  bool conected;
  bool isMeasurement;
};
#endif