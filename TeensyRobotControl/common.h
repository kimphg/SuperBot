#ifndef COMMON_H
#define COMMON_H

//========================================================================================================================//
//#define USE_PWM_RX
#define PPM_Pin 21
//This file contains all necessary functions and code used for radio communication to avoid cluttering the main code

unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6; 
unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
int ppm_counter = 0;
unsigned long time_ms = 0;
//=========================================================================================//
//HELPER FUNCTIONS
void getPPM();
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
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}
//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Version: Beta 1.2
void readAndPrintSpeed(String motorLabel, BLVD20KM_asukiaaa* motor) {
  uint16_t speed;
  auto result = motor->readSpeed(&speed);
  Serial.print(motorLabel);
  Serial.print(" ");
  if (result == 0) {
    Serial.println("Speed is " + String(speed));
  } else {
    Serial.println("Cannot read speed. E:" + String(result) + " " +
                   BLVD20KM_asukiaaa::getStrOfError(result));
  }
}

void readAndPrintAlarm(String motorLabel, BLVD20KM_asukiaaa* motor) {
  uint16_t alarmState;
  auto result = motor->readAlarm(&alarmState);
  Serial.print(motorLabel);
  Serial.print(" ");
  if (result == 0) {
    Serial.println("Current alarm:0x" + String(alarmState, HEX) + " " +
                   BLVD20KM_asukiaaa::getStrOfAlarm(alarmState));
  } else {
    Serial.println("Cannot read alarm. E:" + String(result) + " " +
                   BLVD20KM_asukiaaa::getStrOfError(result));
  }
}

void radioSetup() {
  //PPM Receiver 
  #if defined USE_PPM_RX
    //Declare interrupt pin
    pinMode(PPM_Pin, INPUT_PULLUP);
    delay(20);
    //Attach interrupt and point to corresponding ISR function
    attachInterrupt(digitalPinToInterrupt(PPM_Pin), getPPM, CHANGE);

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
  
  return returnPWM;
}



//========================================================================================================================//



//INTERRUPT SERVICE ROUTINES (for reading PWM and PPM)

void getPPM() {
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


#endif // COMMON_H
