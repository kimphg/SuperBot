// #include "wiring.h"
#ifndef COMMON_H
#define COMMON_H
// #define SIMULATION

#define S_IMU     Serial4
#define S_SENSORS Serial2
#define S_MOTORS  Serial3
#define S_COMMAND Serial1
#define S_DEBUG   Serial5
#ifdef SIMULATION
  #define S_DEBUG Serial
  #define RS485_PPU_PSU Serial
#endif
#define DT_CONTROL 0.02 //50hz control loop
#define PI2 6.2831853
#define PI 3.1415926535897932384626433832795

// #define degrees(rad) ((rad)*57.295779513)


// #define radians(deg) ((deg)/57.295779513)


#define DEBUG
#ifdef DEBUG
#define DPRINT(...)    S_DEBUG.print(__VA_ARGS__)
//OR, #define DPRINT(args...)    Serial.print(args)
#define DPRINTLN(...)  S_DEBUG.println(__VA_ARGS__)
#define DPRINTF(...)    S_DEBUG.print(F(__VA_ARGS__))
#define DPRINTLNF(...) S_DEBUG.println(F(__VA_ARGS__)) //printing text using the F macro
#define DBEGIN(...)    S_DEBUG.begin(__VA_ARGS__)

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
#define PI_NHAN2                    6.2831853072
#define PI_CHIA2                    1.5707963268
#define DEG_RAD 57.295779513f
float ConvXYtoAngle(double x, double y);
uint16_t gen_crc16(const uint8_t *data, uint16_t size);
void setupBlink(int numBlinks, int upTime, int downTime);
void indicateErrorLed(int errorCode) ;
void radioSetup() ;
uint8_t calcCS8(uint8_t* startbyte, uint8_t len);
void blink(int n);
bool isPrintable(uint8_t ch);
std::vector<String> splitString(String input,char sep);
#endif  // COMMON_H
