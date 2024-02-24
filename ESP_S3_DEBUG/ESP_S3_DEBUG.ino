#include "WiFi.h"
#include "AsyncUDP.h"

const char *ssid = "RobotDebug1";
const char *password = "12345678";
bool connected=false;
AsyncUDP udp;
const char * udpAddress = "192.168.4.2";
const int udpPort = 3333;
void setup()
{
  // pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(2000000);
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
  // WiFi.onEvent(WiFiEvent);
  
  // server.begin();
  connected=true;
  Serial.println("Server started");
    if(udp.listen(1234)) {
        Serial.print("UDP Listening on IP: ");
        Serial.println(WiFi.localIP());
        udp.onPacket([](AsyncUDPPacket packet) {
            Serial.write(packet.data(), packet.length());
        });
    }
}
void sendPacket(uint8_t* data,int len)
{
    // udp.beginPacket(udpAddress,udpPort);
    // udp.write(data,len);
    data[len]=0;
    udp.broadcastTo((char*)data, 3333);
    // Serial.println((char*)data);
    // udp.endPacket();
}
uint8_t inputBuff[500];
uint8_t buffIndex=0;
uint8_t lastByte =0;
void loop()
{
     while(Serial.available())
  {
    uint8_t bytein = Serial.read();
    inputBuff[buffIndex]=bytein; 
    buffIndex++;
    if(buffIndex>=500)buffIndex=0;
    // if((bytein=='$')&&(lastByte=='!'))
    // {
    //   inputBuff[0]='!';
    //   inputBuff[1]='$';
    //   buffIndex=2;
    // }
    // else 
    if (bytein=='@'){
        sendPacket( inputBuff, buffIndex+1);
        buffIndex=0;
    }
    lastByte = bytein;
       
  } 
    //Send broadcast
    // udp.broadcast("ROBOT_PING");
}