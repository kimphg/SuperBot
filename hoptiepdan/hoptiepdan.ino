/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  This example code is in the public domain.
 */

// Pin 13 has an LED connected on most Arduino boards.
// Pin 11 has the LED on Teensy 2.0
// Pin 6  has the LED on Teensy++ 2.0
// Pin 13 has the LED on Teensy 3.0
// give it a name:
int led1 = 26;
int led2 = 25;
int led3 = 21;
int led4 = 16;
int laser =27;
int trig=33;
int dcmotor=14;int high_volt=0;
// the setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(laser, OUTPUT);
  pinMode(trig, INPUT);
  pinMode(dcmotor, OUTPUT);
  digitalWrite(dcmotor, LOW); 
  
}
void onTrig()
{  
  digitalWrite(led1, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(led2, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(led3, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(led4, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(laser, HIGH); 
  digitalWrite(high_volt, HIGH); 
  digitalWrite(dcmotor, HIGH); 
  delay(50);
  offTrig();
   
  }
  void offTrig()
{
    digitalWrite(led1, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(led2, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(led3, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(led4, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(laser, LOW); 
  digitalWrite(high_volt, LOW); 
  digitalWrite(dcmotor, LOW); 
  delay(100);
  }
// the loop routine runs over and over again forever:
void loop() {
  while (digitalRead(trig)!=0){};
  onTrig();
  while (digitalRead(trig)!=0){};
  onTrig();
  while (digitalRead(trig)!=0){};
  onTrig();
  while (digitalRead(trig)==0){};
}
