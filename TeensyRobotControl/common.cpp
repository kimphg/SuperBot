#include "common.h"
// unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
int ppm_counter = 0;
unsigned long time_ms = 0;
float ConvXYtoAngle(double x, double y)
{
  float azi=0;
    if(!y)
    {
        azi = x>0? PI_CHIA2:(PI_NHAN2-PI_CHIA2);
        
    }
    else
    {
        azi = atanf(x/y);
        if(y<0)azi+=PI;
        if(azi<0)azi += PI_NHAN2;
    }
  return degrees(azi);
}
#define CRC16 0x8005
float getRom(int id)
{
  return bytes2float(id*4);
}
void setRom(int id,float value)
{
  float2bytes(id*4,value);
  //  EEPROM.write(id*4);
}
uint16_t gen_crc16(const uint8_t *data, uint16_t size)
{
    uint16_t out = 0;
    int bits_read = 0, bit_flag;

    /* Sanity check: */
    if(data == NULL)
        return 0;

    while(size > 0)
    {
        bit_flag = out >> 15;

        /* Get next bit: */
        out <<= 1;
        out |= (*data >> bits_read) & 1; // item a) work from the least significant bits

        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }

        /* Cycle check: */
        if(bit_flag)
            out ^= CRC16;

    }

    // item b) "push out" the last 16 bits
    int i;
    for (i = 0; i < 16; ++i) {
        bit_flag = out >> 15;
        out <<= 1;
        if(bit_flag)
            out ^= CRC16;
    }

    // item c) reverse the bits
    uint16_t crc = 0;
    i = 0x8000;
    int j = 0x0001;
    for (; i != 0; i >>=1, j <<= 1) {
        if (i & out) crc |= j;
    }

    return crc;
}
void ConvXYToPolar(float x, float y, float* azi, float* range)
{
    if(abs(y)<0.001)
    {
        *azi = x>0? PI_CHIA2:(PI_NHAN2-PI_CHIA2);
        *azi = *azi*DEG_RAD;
        *range = abs(x);
    }
    else
    {
        *azi = atanf(x/y);
        if(y<0)*azi+=PI;
        if(*azi<0)*azi += PI_NHAN2;
        *range = sqrt(x*x+y*y);
        *azi = *azi*DEG_RAD;
    }

}
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
    digitalWrite(PIN_OUT_2, LOW);
    digitalWrite(PIN_OUT_1, LOW);
    delay(200);
    digitalWrite(PIN_OUT_2, HIGH);
    digitalWrite(PIN_OUT_1, HIGH);
    delay(200);
  }
    digitalWrite(PIN_OUT_2, LOW);
    digitalWrite(PIN_OUT_1, LOW);
    delay(1000);
}

bool isPrintable(uint8_t ch)
{
    if(ch>=0x21&&ch<=0x7e)return true;
    if(ch==0x0d||ch==0x0a)return true;
    return false;
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
float bytes2float(int id)
{
  float output;
  uint8_t* valueAddr = (uint8_t*)&output;
  for(int i=0;i<4;i++)
  {
    *(valueAddr+i)=EEPROM.read(id*4+i);
  }
  return output;
}
void float2bytes(int id, float value)
{
  uint8_t* valueAddr = (uint8_t*)(&value);
  for(int i=0;i<4;i++)
  {
    EEPROM.write((id*4+i), *(valueAddr+i));
  }

}
uint8_t calcCS8(uint8_t* startbyte, uint8_t len)
{
  int cs = 0;
  for (int i = 0; i < len; i++) {
    cs ^= startbyte[i];
  }
  return cs;
}
uint8_t calcMinus(uint8_t* startbyte, uint8_t len)
{
  uint8_t cs = 0;
  for (int i = 0; i < len; i++) {
    cs -= startbyte[i];
  }
  return cs;
}
uint8_t gencrc8(uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  size_t i;
  for (i = 0; i < len; i++) {
    crc ^= data[i];

  }
  return crc;
}
bool crc8check(String input)
{
      int csPos = input.lastIndexOf(',');
    
    if (csPos <= 0) return 0;
    int crc = gencrc8((uint8_t*)input.c_str(), csPos + 1);
    int realCRC = input.substring(csPos+1,input.length()).toInt();
    if(crc!=realCRC)
    {
        Serial.print("crc fail:");
        Serial.println(input);
        Serial.println(crc);
        Serial.println(input.substring(csPos,input.length()));
        return 0;
    }
    return 1;
}
std::vector<String> splitString(String input,char sep)
{
  std::vector<String> tokens;
  if(input.length()<1)return tokens;
  
  int last_sep_pos=0;
  while(1)
  {
    int sep_pos = input.indexOf(sep,last_sep_pos);
    if(sep_pos<0){
      String token = input.substring(last_sep_pos,input.length());
      tokens.push_back(token);
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
