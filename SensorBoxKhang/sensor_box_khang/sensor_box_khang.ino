
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// the media access control (ethernet hardware) address for the shield:
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  
//the IP address for the shield:
byte ip[] = { 192, 168, 100, 10 };  
char ipRemote []      = "192.168.100.15"; 
int portRemote        = 2021;
IPAddress myDns(192, 168, 100, 1);
IPAddress gateway(192, 168, 100, 1);
IPAddress subnet(255, 255, 255, 0);
unsigned int localPort = 8888;
// buffers for receiving and sending data
#define UDP_TX_PACKET_MAX_SIZE 500
EthernetUDP Udp;
char inBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
char outBuffer[50];
void setup() {
  // put your setup code here, to run once:
  Ethernet.init(10);
  Ethernet.begin(mac, ip, myDns, gateway, subnet);
  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
  Serial.begin(115200);
 // start UDP
  Udp.begin(localPort);
}
long randNumber;
void loop() {
  // put your main code here, to run repeatedly:
 delay(200);
 randNumber = random(300);
 int temp = randNumber/30.0+15.0;
 int hum = randNumber/20.0+50.0;
 int move = randNumber>250;
 
 Serial.print("$SENS,ID,");
 Serial.print("1");
 Serial.print(",TEMP,");
 Serial.print(temp);
 Serial.print(",HUM,");
 Serial.print(hum);
 Serial.print(",MOV,");
 Serial.print(move);
 Serial.print(",#\n");
 // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    for (int i=0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    Udp.read(inBuffer, UDP_TX_PACKET_MAX_SIZE);
    Serial.println("Contents:");
    Serial.println(inBuffer);

    // send a reply to the IP address and port that sent us the packet we received
    
  }
  outBuffer[0]=0xf1;
  memcpy(&outBuffer[1],(char*)&temp,4);
  memcpy(&outBuffer[5],(char*)&hum,4);
  memcpy(&outBuffer[9],(char*)&move,4);
  Udp.beginPacket(ipRemote,portRemote);
  Udp.write(outBuffer,13);
  Udp.endPacket();
}
