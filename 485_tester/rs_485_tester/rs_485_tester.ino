

#include <SoftwareSerial.h>
const byte rxPin = 8;
const byte txPin = 7;

// Set up a new SoftwareSerial object
SoftwareSerial mySerial (rxPin, txPin);
void setup() {
  Serial.begin(57600);
  mySerial.begin(57600);
  delay(10);
  pinMode(13,OUTPUT);
}

void loop() {
  for (long int baud=4800;baud<=200000;baud*=2)
  {
  int nsend = 0,nres=0;
  Serial.begin(baud);
  mySerial.begin(baud);
  delay(10);
  while(Serial.available())
    {
      unsigned char in = Serial.read();
    }
  for(unsigned char i=0;i<=254;i++)
 {
    mySerial.write(i);
    mySerial.flush();
    nsend++;
    delay(1);
    if(Serial.available())
    {
      unsigned char in = Serial.read();
      if(in==i)nres++;
    }
 }
 while(mySerial.available())
    {
      unsigned char in = mySerial.read();
    }
  for(unsigned char i=0;i<=254;i++)
 {
    Serial.write(i);
    Serial.flush();
    nsend++;
    delay(1);
    if(mySerial.available())
    {
      unsigned char in = mySerial.read();
      if(in==i)nres++;
    }
 }
 Serial.begin(57600);
  // mySerial.begin(57600);
 delay(10);
 Serial.print('\n');
 Serial.print(baud);
 Serial.print(':');
 Serial.print(nres);
 Serial.print('/');
 Serial.print(nsend);
  
 if(nres==nsend)
  digitalWrite(13,HIGH);
  else digitalWrite(13,LOW);
  delay (50);
  }
}
