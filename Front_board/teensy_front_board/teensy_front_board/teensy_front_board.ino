
#define LED_1 23
#define LED_2 22
#define LED_3 4
#define LED_4 5
#define LED_5 9
#define LED_6 10
void readSonic()
{
  while(Serial5.available())
  {
    Serial.write(Serial5.read());
  }
}
void setup() {
  Serial5.begin(115200);
  // put your setup code here, to run once:
  pinMode(LED_1,OUTPUT);
  pinMode(LED_2,OUTPUT);
  pinMode(LED_3,OUTPUT);
  pinMode(LED_4,OUTPUT);
  pinMode(LED_5,OUTPUT);
  pinMode(LED_6,OUTPUT);
  analogWrite(LED_3,200);
  analogWrite(LED_4,200);
  analogWrite(LED_5,200);
  analogWrite(LED_6,200);

}

void loop() {
  readSonic();
  delay(1);
  // put your main code here, to run repeatedly:
  //   analogWrite(LED_3,LOW);
  // analogWrite(LED_4,LOW);
  // analogWrite(LED_5,LOW);
  // analogWrite(LED_6,LOW);
  // delay(1000);
  // digitalWrite(LED_1,HIGH);
  // digitalWrite(LED_2,HIGH);

  delay(1000);
  
}
