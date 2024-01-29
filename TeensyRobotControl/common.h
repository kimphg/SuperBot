#ifndef COMMON_H
#define COMMON_H
// #define SIMULATION
#ifdef SIMULATION
  #define RS485_IMU Serial1
  #define RS485_SENS Serial2
  #define RS485_MOTORS Serial
  #define RS485_COM_INPUT Serial
  #define DEBUG_TELEMETRY Serial
#else
  #define RS485_IMU Serial1
  #define RS485_SENS Serial2
  #define RS485_MOTORS Serial3
  #define RS485_COM_INPUT Serial4
  #define DEBUG_TELEMETRY Serial
#endif
#define DT_CONTROL 0.02 //50hz control loop
#define PI2 6.2831853
#ifndef PI
  #define PI 3.1415926535897932384626433832795
#endif
#define degrees(x) ((x)*57.295779513)
#define radians(x) ((x)/57.295779513)
#define DEBUG
#ifdef DEBUG
#define DPRINT(...)    Serial.print(__VA_ARGS__)
//OR, #define DPRINT(args...)    Serial.print(args)
#define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#define DPRINTF(...)    Serial.print(F(__VA_ARGS__))
#define DPRINTLNF(...) Serial.println(F(__VA_ARGS__)) //printing text using the F macro
#define DBEGIN(...)    Serial.begin(__VA_ARGS__)
#else
#define DPRINT(...)     //blank line
#define DPRINTLN(...)   //blank line
#define DPRINTF(...)    //blank line
#define DPRINTLNF(...)  //blank line
#define DBEGIN(...)     //blank line
#endif
#include <Arduino.h>
#include <vector>

#define PPM_Pin 10
//This file contains all necessary functions and code used for radio communication to avoid cluttering the main code

// void getPPM() ;

void setupBlink(int numBlinks, int upTime, int downTime) ;
void indicateErrorLed(int errorCode) ;
void radioSetup() ;
uint8_t calcCS8(uint8_t* startbyte, uint8_t len);
void blink(int n);
std::vector<String> splitString(String input);
#endif  // COMMON_H
