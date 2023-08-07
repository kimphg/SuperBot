// Pin 13 has an LED connected on most Arduino boards.
// Pin 11 has the LED on Teensy 2.0
// Pin 6  has the LED on Teensy++ 2.0
// Pin 13 has the LED on Teensy 3.0
// give it a name:


#include<WiFi.h>
//#include <WiFiUdp.h>

//WiFiUDP udp;

char packetBuffer[255];
unsigned int localPort = 60110;

int buttonState = 0;
int m_con = 0;
int m_t = 0;
const char* ssid = "NC_TK_HT_V3_2";         //Tên mạng Wifi mà Socket server của bạn đang kết nối
const char* password = "236hoangquocviet";  //Pass mạng wifi ahihi, anh em rãnh thì share pass cho mình với.

int led1 = 26;
int led2 = 25;
int led3 = 21;
int led4 = 16;
<<<<<<< Updated upstream
int laser =27;
int trig=33;
int dcmotor=14;
int high_volt=0;
=======
int laser = 27;
int trig = 33;
int dcmotor = 14;
int high_volt = 16;
>>>>>>> Stashed changes
// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);
  Serial.print("\nWifi connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("\nWifi failure");
    WiFi.begin(ssid, password);
    delay(100);
  }
   {
    Serial.print("\nWifi OK");
    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
  }
  // initialize the digital pin as an output.
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
//  pinMode(led4, OUTPUT);
  pinMode(laser, OUTPUT);
  pinMode(trig, INPUT);
  pinMode(dcmotor, OUTPUT);
<<<<<<< Updated upstream
  digitalWrite(dcmotor, LOW); 
  
}
void onTrig()//
{  
=======
  digitalWrite(dcmotor, LOW);

}
void onTrig()//
{
>>>>>>> Stashed changes
  digitalWrite(led1, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(led2, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(led3, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(led4, HIGH);   // turn the LED on (HIGH is the voltage level)
<<<<<<< Updated upstream
  digitalWrite(laser, HIGH); 
  digitalWrite(high_volt, HIGH); 
  digitalWrite(dcmotor, HIGH); 
  delay(100);
  offTrig();
   
}
  void offTrig()
=======
  digitalWrite(laser, HIGH);
  digitalWrite(high_volt, HIGH);
  digitalWrite(dcmotor, HIGH);

}

void offTrig()
>>>>>>> Stashed changes
{
  digitalWrite(led1, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(led2, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(led3, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(led4, LOW);   // turn the LED on (HIGH is the voltage level)
<<<<<<< Updated upstream
  digitalWrite(laser, LOW); 
  digitalWrite(high_volt, LOW); 
  digitalWrite(dcmotor, LOW); 
  delay(200);
  }
// the loop routine runs over and over again forever:
void loop() {
  while (digitalRead(trig)==0){};
  onTrig();
  while (digitalRead(trig)!=0){};
=======
  digitalWrite(laser, LOW);
  digitalWrite(high_volt, LOW);
  digitalWrite(dcmotor, LOW);

}
// the loop routine runs over and over again forever:
int dem_co_bop = 0;
int dem_co_nha = 0;
uint8_t dataPacket[] = {'0', '5', '1', 0x00};
void loop() {

  m_t++;
  if (digitalRead(trig) == 0)
  {
    dem_co_bop = 0;
    dem_co_nha++;
  }
  if (digitalRead(trig) != 0)
  { dem_co_bop++;
    dem_co_nha = 0;
  }
  if (dem_co_bop > 10000)
  {
    onTrig();
//    udp.beginPacket("192.168.1.239", 60111);
//    udp.write('0');
//    udp.write('5');
//    udp.write('1');
//    udp.endPacket();
    delay(70);
    offTrig();
    delay(100);
    dem_co_bop = 0;
  }


  // while (digitalRead(trig)!=0){
  //   m_con++;
  //   if(m_con>1000){
  //   onTrig();
  //    m_con=0;
  //   }
  //  };
>>>>>>> Stashed changes

}
