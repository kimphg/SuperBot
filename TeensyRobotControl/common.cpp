#include "common.h"
// unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
int ppm_counter = 0;
unsigned long time_ms = 0;
// void getPPM()
// {
//   // Serial.println("getppm");
//   unsigned long dt_ppm;
//   int trig = digitalRead(PPM_Pin);
//   if (trig == 1) {  //only care about rising edge
//     dt_ppm = micros() - time_ms;
//     time_ms = micros();


//     if (dt_ppm > 5000) {  //waiting for long pulse to indicate a new pulse train has arrived
//       ppm_counter = 0;
//     }

//     if (ppm_counter == 1) {  //first pulse
//       channel_1_raw = dt_ppm;
//     }

//     if (ppm_counter == 2) {  //second pulse
//       channel_2_raw = dt_ppm;
//     }

//     if (ppm_counter == 3) {  //third pulse
//       channel_3_raw = dt_ppm;
//     }

//     if (ppm_counter == 4) {  //fourth pulse
//       channel_4_raw = dt_ppm;
//     }

//     if (ppm_counter == 5) {  //fifth pulse
//       channel_5_raw = dt_ppm;
//     }

//     if (ppm_counter == 6) {  //sixth pulse
//       channel_6_raw = dt_ppm;
//     }

//     ppm_counter = ppm_counter + 1;
//   }
// }

void setupBlink(int numBlinks, int upTime, int downTime)
{
  //DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j <= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
    digitalWrite(13, LOW);
  }
}
void blink(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
  }
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
}


void radioSetup()
{
    //Declare interrupt pin
    pinMode(PPM_Pin, INPUT_PULLUP);
    delay(20);
    //Attach interrupt and point to corresponding ISR function
    // attachInterrupt(digitalPinToInterrupt(PPM_Pin), getPPM, CHANGE);
    Serial.println("radioSetup to PPM mode");
    Serial.flush();

}

uint8_t calcCS8(uint8_t* startbyte, uint8_t len)
{
  int cs = 0;
  for (int i = 0; i < len; i++) {
    cs ^= startbyte[i];
  }
  return cs;
}

std::vector<String> splitString(String input)
{
  std::vector<String> tokens;
  if(input.length()<1)return tokens;
  
  int last_sep_pos=0;
  while(1)
  {
    int sep_pos = input.indexOf(',',last_sep_pos);
    if(sep_pos<0){
      break;
    }
    String token = input.substring(last_sep_pos,sep_pos);
    last_sep_pos = sep_pos+1;
    tokens.push_back(token);
  }
  // DPRINTLN(tokens.size());
  // for(unsigned int i=0;i<tokens.size();i++)
  // {
  //   DPRINTLN(tokens[i]);
  // }
  return tokens;
}
