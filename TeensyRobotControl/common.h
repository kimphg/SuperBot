#ifndef COMMON_H
#define COMMON_H
//Uncomment only one receiver type
//#define USE_PWM_RX
#define USE_PPM_RX
//#define USE_SBUS_RX
#define BAT_VOL_SENS A1
//Uncomment only one ESC type
// #define USE_PWM_ESC
//#define USE_ONESHOT_ESC
#define USE_BLVM_MODBUS
//Uncomment only one IMU
// #define USE_MPU6050_I2C //default
//#define USE_MPU9250_SPI

//Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS //default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

//Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G //default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G



//========================================================================================================================//


#if defined USE_SBUS_RX
#include "src/SBUS/SBUS.h"   //sBus interface
#endif

#if defined USE_MPU6050_I2C
#include "src/MPU6050/MPU6050.h"
MPU6050 mpu6050;
#elif defined USE_MPU9250_SPI
#include "src/MPU9250/MPU9250.h"
MPU9250 mpu9250(SPI2, 36);

#endif
#include <Arduino.h>


//========================================================================================================================//



//Setup gyro and accel full scale value selection and scale factor

#if defined USE_MPU6050_I2C
#define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16
#elif defined USE_MPU9250_SPI
#define GYRO_FS_SEL_250    mpu9250.GYRO_RANGE_250DPS
#define GYRO_FS_SEL_500    mpu9250.GYRO_RANGE_500DPS
#define GYRO_FS_SEL_1000   mpu9250.GYRO_RANGE_1000DPS
#define GYRO_FS_SEL_2000   mpu9250.GYRO_RANGE_2000DPS
#define ACCEL_FS_SEL_2     mpu9250.ACCEL_RANGE_2G
#define ACCEL_FS_SEL_4     mpu9250.ACCEL_RANGE_4G
#define ACCEL_FS_SEL_8     mpu9250.ACCEL_RANGE_8G
#define ACCEL_FS_SEL_16    mpu9250.ACCEL_RANGE_16G
#endif

#if defined GYRO_250DPS
#define GYRO_SCALE GYRO_FS_SEL_250
#define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
#define GYRO_SCALE GYRO_FS_SEL_500
#define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
#define GYRO_SCALE GYRO_FS_SEL_1000
#define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
#define GYRO_SCALE GYRO_FS_SEL_2000
#define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
#define ACCEL_SCALE ACCEL_FS_SEL_2
#define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
#define ACCEL_SCALE ACCEL_FS_SEL_4
#define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
#define ACCEL_SCALE ACCEL_FS_SEL_8
#define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
#define ACCEL_SCALE ACCEL_FS_SEL_16
#define ACCEL_SCALE_FACTOR 2048.0
#endif

//========================================================================================================================//
//#define USE_PWM_RX
#define PPM_Pin 20
//This file contains all necessary functions and code used for radio communication to avoid cluttering the main code

unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6; 
unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
int ppm_counter = 0;
unsigned long time_ms = 0;

void getPPM() {
  // Serial.println("getppm");
  unsigned long dt_ppm;
  int trig = digitalRead(PPM_Pin);
  if (trig==1) { //only care about rising edge
    dt_ppm = micros() - time_ms;
    time_ms = micros();

    
    if (dt_ppm > 5000) { //waiting for long pulse to indicate a new pulse train has arrived
      ppm_counter = 0;
    }
  
    if (ppm_counter == 1) { //first pulse
      channel_1_raw = dt_ppm;
    }
  
    if (ppm_counter == 2) { //second pulse
      channel_2_raw = dt_ppm;
    }
  
    if (ppm_counter == 3) { //third pulse
      channel_3_raw = dt_ppm;
    }
  
    if (ppm_counter == 4) { //fourth pulse
      channel_4_raw = dt_ppm;
    }
  
    if (ppm_counter == 5) { //fifth pulse
      channel_5_raw = dt_ppm;
    }
  
    if (ppm_counter == 6) { //sixth pulse
      channel_6_raw = dt_ppm;
    }
    
    ppm_counter = ppm_counter + 1;
  }
}

//=========================================================================================//
//HELPER FUNCTIONS
float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *((float*)&i);
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}
//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Version: Beta 1.2

void setupBlink(int numBlinks, int upTime, int downTime) {
  //DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j <= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
    digitalWrite(13, LOW);
  }
}
void indicateErrorLed(int errorCode)
{
  if(errorCode==0)
  {
    digitalWrite(13, LOW);
    delay(500);
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(500);

  }
    else
    {
      
      setupBlink(errorCode, 300, 100); //numBlinks, upTime (ms), downTime (ms)
      delay(2000);
    }
}
void radioSetup() {
  //PPM Receiver 
  #if defined USE_PPM_RX
    //Declare interrupt pin
    pinMode(PPM_Pin, INPUT);
    delay(20);
    //Attach interrupt and point to corresponding ISR function
    attachInterrupt(digitalPinToInterrupt(PPM_Pin), getPPM, CHANGE);
    Serial.println("radioSetup to PPM mode");Serial.flush();
  //PWM Receiver
  #elif defined USE_PWM_RX
    //Declare interrupt pins 
    pinMode(ch1Pin, INPUT_PULLUP);
    pinMode(ch2Pin, INPUT_PULLUP);
    pinMode(ch3Pin, INPUT_PULLUP);
    pinMode(ch4Pin, INPUT_PULLUP);
    pinMode(ch5Pin, INPUT_PULLUP);
    pinMode(ch6Pin, INPUT_PULLUP);
    delay(20);
    //Attach interrupt and point to corresponding ISR functions
    attachInterrupt(digitalPinToInterrupt(ch1Pin), getCh1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch2Pin), getCh2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch3Pin), getCh3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch4Pin), getCh4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch5Pin), getCh5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch6Pin), getCh6, CHANGE);
    delay(20);

  //SBUS Recevier 
  #elif defined USE_SBUS_RX
    sbus.begin();
    Serial.println("radioSetup to Sbus mode");
  #else
    #error No RX type defined...
  #endif
}

unsigned long getRadioPWM(int ch_num) {
  //DESCRIPTION: Get current radio commands from interrupt routines 
  unsigned long returnPWM;
  
  if (ch_num == 1) {
    returnPWM = channel_1_raw;
  }
  else if (ch_num == 2) {
    returnPWM = channel_2_raw;
  }
  else if (ch_num == 3) {
    returnPWM = channel_3_raw;
  }
  else if (ch_num == 4) {
    returnPWM = channel_4_raw;
  }
  else if (ch_num == 5) {
    returnPWM = channel_5_raw;
  }
  else if (ch_num == 6) {
    returnPWM = channel_6_raw;
  }
  else returnPWM=0;
  
  return returnPWM;
}



//========================================================================================================================//


#endif // COMMON_H
