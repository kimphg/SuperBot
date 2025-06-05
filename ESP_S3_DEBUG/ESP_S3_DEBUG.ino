#include "WiFi.h"
#include "AsyncUDP.h"
#include <HardwareSerial.h>
HardwareSerial SerialPort(1) ;
#define RXD 1
#define TXD 3
const char *ssid = "IMEBOTS2";
const char *password = "20212025";
//note : nối tx2 rx2 trước khi nạp
bool connected=false;
AsyncUDP udp;
const char * udpAddress = "192.168.0.103";
const int udpPort = 3333;
void setup()
{
  // pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(921600);
  SerialPort.begin(921600, SERIAL_8N1, 14, 46);
  Serial.println("Configuring access point...");

  // You can remove the password parameter if you want the AP to be open.
  // a valid password must have more than 7 characters
  if (!WiFi.begin(ssid, password)) {
    log_e("WIFI begin failed.");
    while(1);
  }
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000); // Wait for 1 second
  }
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("My IP address: ");
  Serial.println(myIP);
  // WiFi.onEvent(WiFiEvent);
  
  // server.begin();
  connected=true;
  Serial.println("Server started");
    if(udp.listen(1234)) {
        Serial.print("UDP Listening on IP: ");
        Serial.println(WiFi.localIP());
        udp.onPacket([](AsyncUDPPacket packet) {
            SerialPort.write(packet.data(), packet.length());
            SerialPort.flush();
        });
    }
}
void sendPacket(uint8_t* data,int len)
{
    // udp.beginPacket(udpAddress,udpPort);
    // udp.write(data,len);
    data[len]=0;
    udp.broadcastTo((char*)data, 3333);
    Serial.println((char*)data);
    // udp.endPacket();
}
uint8_t inputBuff[500];
uint8_t buffIndex=0;
uint8_t lastByte =0;
void loop()
{
     while(SerialPort.available())
  {
    uint8_t bytein = SerialPort.read();
    inputBuff[buffIndex]=bytein; 
    buffIndex++;
    if(buffIndex>=500)
    {
      sendPacket( inputBuff, 500);
      buffIndex=0;
    }
    else if (bytein=='\n'){
        sendPacket( inputBuff, buffIndex+1);
        buffIndex=0;
    }
    lastByte = bytein; 
  } 
  if(WiFi.status() != WL_CONNECTED)
  {
    setup();
  }
    //Send broadcast
    // udp.broadcast("ROBOT_PING");
}