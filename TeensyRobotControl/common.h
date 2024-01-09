#ifndef COMMON_H
#define COMMON_H
#define RS485_IMU Serial1
#define RS485_SENS Serial2
#define RS485_MOTORS Serial3
#define RS485_COM_INPUT Serial4
#define DEBUG_TELEMETRY Serial
#define DT_CONTROL 0.02 //50hz control loop
#define PI2 6.2831853
#define PI 3.141592653589793


#include <Arduino.h>


#define PPM_Pin 10
//This file contains all necessary functions and code used for radio communication to avoid cluttering the main code

// void getPPM() ;

void setupBlink(int numBlinks, int upTime, int downTime) ;
void indicateErrorLed(int errorCode) ;
void radioSetup() ;

#endif  // COMMON_H
