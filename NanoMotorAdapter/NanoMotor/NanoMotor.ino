//micro controller type: LGT8F328P
#define MEN       2
#define FWD       3
#define ALM_RES   4
#define SPEED     5
#define MB_FREE   6
#define MIN_LIM   7
#define WNG_IN    8
#define ALARM_IN  9
#define MAX_LIM   10
#define LED_1     11
#define DEBUG_TELEMETRY Serial
#define COMMAND_LEN_MAX 100
int output_speed;
int stackLevel=0;
String commandString;
bool isMin,isMax;
void setSpeed(int speed);
void updateCommandBus()
{
  while(DEBUG_TELEMETRY.available())
  {
    uint8_t bytein = DEBUG_TELEMETRY.read();
    if(commandString.length()<COMMAND_LEN_MAX)commandString+=(char)bytein;
    if(bytein=='\n')//end of command
    {
      int dataLen = commandString.length();
      if(commandString.startsWith("TRS="))//angle set command
      {
        float speed = commandString.substring(4,dataLen-1).toFloat();
        setSpeed(speed);
        DEBUG_TELEMETRY.print("Speed set ok:");
        DEBUG_TELEMETRY.println(output_speed);
      }
      commandString = "";
    }
  }
}
uint8_t commandBuff[COMMAND_LEN_MAX];
int commandBuffIndex=0;
void updateBinaryCommand()
{
  while(DEBUG_TELEMETRY.available())
  {
    uint8_t bytein = DEBUG_TELEMETRY.read();
    if(commandBuffIndex<COMMAND_LEN_MAX)commandBuff[commandBuffIndex]=bytein;
    if(bytein=='\n')//end of command
    {
      int dataLen = commandString.length();
      if(commandString.startsWith("TRS="))//angle set command
      {
        float speed = commandString.substring(4,dataLen-1).toFloat();
        setSpeed(speed);
        DEBUG_TELEMETRY.print("Speed set ok:");
        DEBUG_TELEMETRY.println(output_speed);
      }
      commandString = "";
    }
  }
}
void setSpeed(int speed)
{
  output_speed=speed;
  if(isMin)if(output_speed<0)output_speed=0;
  if(isMax)if(output_speed>0)output_speed=0;
  if(speed>0){
    digitalWrite(FWD,HIGH);
    analogWrite(SPEED, 255-output_speed);
  }
  else
  {
    digitalWrite(FWD,LOW);
    analogWrite(SPEED, 255+output_speed);
  }
}
void setup() {
  pinMode(MIN_LIM,INPUT);     
  pinMode(WNG_IN,INPUT);      
  pinMode(ALARM_IN,INPUT);    
  pinMode(MAX_LIM,INPUT);       
  pinMode(LED_1,OUTPUT);      digitalWrite(LED_1,HIGH);
  pinMode(LED_BUILTIN,OUTPUT);digitalWrite(LED_BUILTIN,LOW);
  pinMode(MEN,OUTPUT);        digitalWrite(MEN,LOW);
  pinMode(FWD,OUTPUT);        digitalWrite(FWD,LOW);
  pinMode(ALM_RES,OUTPUT);    digitalWrite(ALM_RES,LOW);
  pinMode(SPEED,OUTPUT);      digitalWrite(SPEED,HIGH);
  pinMode(MB_FREE,OUTPUT);    digitalWrite(MB_FREE,HIGH);
  setSpeed(0);
  Serial.begin(500000);
}

void loop() {
  updateCommandBus();
  isMin = digitalRead(MIN_LIM);
  isMax = digitalRead(MAX_LIM);
DEBUG_TELEMETRY.println("Motor Ready");
}
