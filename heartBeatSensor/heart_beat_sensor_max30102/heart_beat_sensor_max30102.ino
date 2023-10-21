/*
  Optical SP02 Detection (SPK Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 19th, 2016
  https://github.com/sparkfun/MAX30105_Breakout

  This demo shows heart rate and SPO2 levels.

  It is best to attach the sensor to your finger using a rubber band or other tightening 
  device. Humans are generally bad at applying constant pressure to a thing. When you 
  press your finger against the sensor it varies enough to cause the blood in your 
  finger to flow differently which causes the sensor readings to go wonky.

  This example is based on MAXREFDES117 and RD117_LILYPAD.ino from Maxim. Their example
  was modified to work with the SparkFun MAX30105 library and to compile under Arduino 1.6.11
  Please see license file for more info.

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
 
  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/

#include <Wire.h>
//#include <Wire1.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 sen1,sen0;

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read
MbedI2C  wire1(6,7);
MbedI2C wire0(4,5);
void setup()
{
//  Wire.setSDA(4);
//  Wire.setSCL(5);
  Wire.begin() ;
//   wire1.setSDA(6);
//   wire1.setSCL(7);
  wire1.begin();
  Serial.begin(500000); // initialize serial communication at 115200 bits per second:
  Serial1.begin(500000); // initialize serial communication at 115200 bits per second:

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

 // Initialize sensor
  while (!sen0.begin(wire0, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("wire0 was not found. Please check wiring/power."));
     delay(1000);
  }
  // Initialize sensor
  while (!sen1.begin(wire1, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("wire1 was not found. Please check wiring/power."));
     delay(1000);
  }

  // Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  // while (Serial.available() == 0) ; //wait until user presses a key
  // Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  sen1.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  sen0.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  Serial.print("Start in 3");
  delay(1000);
  Serial.print("Start in 2");
  delay(1000);
  Serial.print("Start in 1");
  delay(1000);
  Serial.print("Start record");
}

void loop()
{
  
    while (sen0.available() == false) //do we have new data?
      sen0.check(); //Check the sensor for new data

    uint16_t reddata= sen0.getRed();
    uint16_t irdata= sen0.getIR();
    sen0.nextSample(); //We're finished with this sample so move to next sample
    Serial.print("$");          Serial1.write(0xff);   
    Serial.print(F(","));       Serial1.write(0xaa);
    Serial.print(reddata, DEC); Serial1.write((char*)(&reddata), 2);
//    Serial.print(F(","));       Serial1.print(F(","));
    Serial.print(irdata, DEC);  Serial1.write((char*)(&reddata), 2);
//    Serial.print(F(","));       Serial1.print(F(","));
    while (sen1.available() == false) //do we have new data?
      sen1.check(); //Check the sensor for new data

    reddata= sen1.getRed();
    irdata= sen1.getIR();
    sen1.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(reddata, DEC);
    Serial.print(F(","));
    Serial.print(irdata, DEC);
    Serial.println(",#");
    
    Serial1.print(reddata, DEC);
    Serial1.print(F(","));
    Serial1.print(irdata, DEC);
    Serial1.println(",#");
  
}
