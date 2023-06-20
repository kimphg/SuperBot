#define RXPIN 5
#define TXPIN 4
UART magneto( TXPIN, RXPIN, 0, 0);
byte msg_setup[] = {0x01, 0x03, 0x00, 0x20, 0x00, 0x08, 0x45, 0xC6};
void setup() {
  // put your setup code here, to run once:
  magneto.begin(115200);
  Serial.begin(115200);

}
byte inputBuff[20];
void loop() {
  // put your main code here, to run repeatedly:
  delay(5);
  magneto.write(msg_setup,8);delay(5);
  int i=0;
  while (magneto.available())
  {
    inputBuff[i] = magneto.read();
    i++;
  }
  if((inputBuff[0]==0x01)&&(inputBuff[1]==0x03)&&(inputBuff[2]==0x10))
  {
    float sum=0;
    int max_index=0;
    int max_value=0;
    for(i=3;i<19;i++)
    {
      sum+= (inputBuff[i]);
      if(max_value<inputBuff[i])
      {
        max_value = inputBuff[i];
        max_index=i-3;
      }
      }
      float avrg = sum/16.0;
      if(max_value>(3+avrg*1.5))
      Serial.println(max_index);
      else Serial.println(8);
  }
  
}
