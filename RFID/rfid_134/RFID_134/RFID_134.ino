// #include <SoftwareSerial.h>
#define RXPIN 5
#define TXPIN 4
UART rfidReader( TXPIN, RXPIN,0, 0);
void setup() {
  // put your setup code here, to run once:
//  rfidReader.begin (9600);
  Serial.begin(9600);
  pinMode(8,INPUT);
//  digitalWrite(8,LOW);
  delay(100);
  for(int i=0;i<1000;i++)
  {
    Serial.print("fff");
  }

  rfidReader.begin(9600);
}

void loop() {
  Serial.println(digitalRead(8));
//  digitalWrite(8,LOW);
  delay(100);
//  digitalWrite(8,HIGH);
//  delay(1000);
  // put your main code here, to run repeatedly:
//  while(rfidReader.available())
//  {
//    byte input=rfidReader.read();
//    Serial.write(input);
//////    Serial.print("fff");
//  }
//  if(1)
//  {
//    Serial.print("fff");
//    }

}
