void setup() {
  // put your setup code here, to run once:
 Serial.begin(115200);
}
long randNumber;
void loop() {
  // put your main code here, to run repeatedly:
 delay(200);
 randNumber = random(300);
 int temp = randNumber/30.0+15.0;
 int hum = randNumber/20.0+50.0;
 bool move = randNumber>250;
 Serial.print("$SENS,ID,");
 Serial.print(randNumber/60);
 Serial.print(",TEMP,");
 Serial.print(temp);
 Serial.print(",HUM,");
 Serial.print(hum);
 Serial.print(",MOV,");
 Serial.print(move);
 Serial.print(",#\n");
}
