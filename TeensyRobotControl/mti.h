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
  float roll, pitch, yaw,gyroyaw;
  float gyroX,gyroY,gyroZ,gyroZold;
  float accX,accY,accZ;

} IMUData;
class IMU_driver {
public:
  float yawShift;
  int yawCalcMode ;
  IMU_driver();
  void IMU_init(Stream &porti) ;
  void resetYaw();
  bool Connect() ;
  bool gotoMeasurement();
  inline bool getIsConnected() {
    return isConnected;
  }
  bool gotoConfig() ;
  void updateData() ;
  
  inline IMUData getMeasurement() {
    dataUpdated=false;
    return measurement;
  }
  bool dataUpdated;
  int noMotionCount;
  float gyroZBiasCompensation;
private:
  float gyroZBias;
  
  int gyroZBiasCount;
  unsigned char databuf[BUF_SIZE_IMU];
  int buffIndex;
  
  unsigned char lastbyte;
  IMUData measurement;
  Stream *port;
  bool isConnected;
  bool isMeasurement;
};
#endif