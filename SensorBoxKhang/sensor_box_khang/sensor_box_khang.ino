
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <DHT.h>
#define PIN_DHT       2          // PD2 (INT0)
#define PIN_PIR       4          // PD4
#define PIN_LED_CTRL  A2         // PC2 / ADC2 -> LEDControl (GP2Y10)
#define PIN_DUST_AO   A3         // PC3 / ADC3 -> ANALOG_OUTPUT (GP2Y10)
#define PIN_MQ135     A0         // PC0 / ADC0
#define PIN_NOISE     A1         // PC1 / ADC1
#define DHTTYPE DHT22
#define ETH_RST 4
DHT dht(PIN_DHT, DHTTYPE);
// the media access control (ethernet hardware) address for the shield:
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  
//the IP address for the shield:
byte ip[] = { 192, 168, 100, 10 };  
char ipRemote []      = "192.168.100.221"; 
int portRemote        = 2021;
IPAddress myDns(192, 168, 100, 1);
IPAddress gateway(192, 168, 100, 1);
IPAddress subnet(255, 255, 255, 0);
unsigned int localPort = 8888;
static uint16_t sampleDustRaw() {
  analogWrite(PIN_LED_CTRL, 255);   // bật LED
  delayMicroseconds(280);
  uint16_t val = analogRead(PIN_DUST_AO);
  delayMicroseconds(40);
  analogWrite(PIN_LED_CTRL, 0);     // tắt LED
  delayMicroseconds(9680);          //  10ms chu kỳ
  return val;
}
// buffers for receiving and sending data
#define UDP_TX_PACKET_MAX_SIZE 500
EthernetUDP Udp;
char inBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
char outBuffer[200];
void setup() {
  // put your setup code here, to run once:
  Ethernet.init(10);
  pinMode(ETH_RST,OUTPUT);
  digitalWrite(ETH_RST,HIGH);
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


  // Nếu đọc lỗi DHT, trả NaN – vẫn gửi để phía server biết
  uint16_t dust = sampleDustRaw();        // 0..1023
  uint16_t mq135 = analogRead(PIN_MQ135);
  uint16_t noise = analogRead(PIN_NOISE);
  int pir = digitalRead(PIN_PIR);
  const float Vref = 5.0;
  float dustV  = dust  * Vref / 1023.0;
  float mqV    = mq135 * Vref / 1023.0;
  float noiseV = noise * Vref / 1023.0;

 randNumber = random(300);
 uint16_t temp = dht.readTemperature();
 uint16_t hum = dht.readHumidity();

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
  sprintf(outBuffer, "{\"id\": %d, \"temperature\": %d, \"humidity\": %d, \"pm25\": %d, \"noise\": %d, \"mq135\": %d }", 1, int(temp),int(hum),int(dust),int(noise),int(mq135));
  // outBuffer[0]=0xf1;
  // memcpy(&outBuffer[1],(char*)&temp,4);
  // memcpy(&outBuffer[5],(char*)&hum,4);
  // memcpy(&outBuffer[9],(char*)&move,4);
  Udp.beginPacket(ipRemote,portRemote);
  Udp.write(outBuffer,strlen(outBuffer));
  Udp.endPacket();
}
