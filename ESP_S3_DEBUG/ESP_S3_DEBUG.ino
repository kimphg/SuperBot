/*
  WiFiAccessPoint.ino creates a WiFi access point and provides a web server on it.

  Steps:
  1. Connect to the access point "yourAp"
  2. Point your web browser to http://192.168.4.1/H to turn the LED on or http://192.168.4.1/L to turn it off
     OR
     Run raw TCP "GET /H" and "GET /L" on PuTTY terminal with 192.168.4.1 as IP address and 80 as port

  Created for arduino-esp32 on 04 July, 2018
  by Elochukwu Ifediora (fedy0)
*/

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#define LED_BUILTIN 2   // Set the GPIO pin where you connected your test LED or comment this line out if your dev board has a built-in LED
const char * udpAddress = "192.168.4.2";
const int udpPort = 3333;
// Set these to your desired credentials.
const char *ssid = "RobotDebug1";
const char *password = "12345678";
void WiFiEvent(WiFiEvent_t event);

WiFiUDP udp;
bool connected = false;
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(2000000);
  Serial.println();
  Serial.println("Configuring access point...");

  // You can remove the password parameter if you want the AP to be open.
  // a valid password must have more than 7 characters
  if (!WiFi.softAP(ssid, password)) {
    log_e("Soft AP creation failed.");
    while(1);
  }
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  WiFi.onEvent(WiFiEvent);
  // server.begin();
connected=true;
  Serial.println("Server started");
}
uint8_t inputBuff[200];
uint8_t buffIndex=0;
uint8_t lastByte =0;
void sendPacket(uint8_t* data,int len)
{
    udp.beginPacket(udpAddress,udpPort);
    udp.write(data,len);
    udp.endPacket();
}
void loop() {
  while(Serial.available())
  {
    uint8_t bytein = Serial.read();
    inputBuff[buffIndex]=bytein; 
    buffIndex++;
    if(buffIndex>=100)buffIndex=0;
    if((bytein=='$')&&(lastByte=='!'))
    {
      inputBuff[0]='!';
      inputBuff[1]='$';
      buffIndex=2;
    }
    else if (bytein=='#'){
        sendPacket( inputBuff, buffIndex+1);
    }
    lastByte = bytein;
       
  }
    if(connected){ 
  }
  //Wait for 1 second
  delay(1000);
  
}
//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
      default: break;
    }
}

