#define RXPIN 5
#define TXPIN 4
UART rfidReader( TXPIN, RXPIN,0, 0);
byte msg_setup[] = {0x01, 0x06, 0x00, 0x2F, 0x00, 0x05, 0x78, 0x00};
void setup() {
  // put your setup code here, to run once:
  rfidReader.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  rfidReader.write(msg_setup,8);
  delay(500);
}
