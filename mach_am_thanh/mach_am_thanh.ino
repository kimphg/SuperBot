/***************************************************
DFPlayer - A Mini MP3 Player For Arduino
 <https://www.dfrobot.com/product-1121.html>
 
 ***************************************************
 This example shows the basic function of library for DFPlayer.
 
 Created 2016-12-07
 By [Angelo qiao](Angelo.qiao@dfrobot.com)
 
 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
 1.Connection and Diagram can be found here
 <https://www.dfrobot.com/wiki/index.php/DFPlayer_Mini_SKU:DFR0299#Connection_Diagram>
 2.This code is tested on Arduino Uno, Leonardo, Mega boards.
 ****************************************************/
#define LED_INDICATOR 4
#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"
#define ARDUINO_AVR_UNO
#if (defined(ARDUINO_AVR_UNO) || defined(ESP8266))  // Using a soft serial port
#include <SoftwareSerial.h>

SoftwareSerial softSerial(/*rx =*/10, /*tx =*/9);
#define FPSerial softSerial
#else
#define FPSerial Serial1
#endif
SoftwareSerial RF_rx(3, 4);
#define BT1 A0
#define BT2 A1
#define BT3 A2
#define BT4 A3
#define BT5 A4
#define BT6 6
#define BT7 7
#define BT8 8
void bt_init() {
  pinMode(BT1, INPUT_PULLUP);
  pinMode(BT2, INPUT_PULLUP);
  pinMode(BT3, INPUT_PULLUP);
  pinMode(BT4, INPUT_PULLUP);
  pinMode(BT5, INPUT_PULLUP);
  pinMode(BT6, INPUT_PULLUP);
  pinMode(BT7, INPUT_PULLUP);
  pinMode(BT8, INPUT_PULLUP);
}
int check_btn() {
  if (!digitalRead(BT1)) return 1;
  if (!digitalRead(BT2)) return 8;
  if (!digitalRead(BT3)) return 5;
  if (!digitalRead(BT4)) return 2;
  if (!digitalRead(BT5)) return 3;
  if (!digitalRead(BT6)) return 6;
  if (!digitalRead(BT7)) return 4;
  if (!digitalRead(BT8)) return 7;
  return 0;
}
DFRobotDFPlayerMini myDFPlayer;
int volume = 25;
int countDown =0;
void printDetail(uint8_t type, int value);
bool sdok = false;
int combine_1st=0;
int combine_2nd=0;
int combine_3rd=0;
void combination_reset()
{
  combine_1st=0;
  combine_2nd=0;
  combine_3rd=0;
}
void combination_add(int command)
{
  combine_1st=combine_2nd;
  combine_2nd=combine_3rd;
  combine_3rd=command;
}
int combination_getid()
{
  int id = 0;
  if(combine_1st)id+=(0x01<<(combine_1st-1));
  if(combine_2nd)id+=(0x01<<(combine_2nd-1));
  if(combine_3rd)id+=(0x01<<(combine_3rd-1));
  Serial.print(" com: ");
  Serial.print(combine_1st+1);
  Serial.print(combine_2nd+1);
  Serial.println(combine_3rd+1);
  return id;
}
void setup() {
  bt_init();
  RF_rx.begin(9600);
  FPSerial.begin(9600);


  Serial.begin(115200);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini   "));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(FPSerial, /*isACK = */ true, /*doReset = */ true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    // while(true){
    //   delay(0); // Code to compatible with ESP8266 watch dog.
    // }
  } else sdok = true;
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.volume(volume);  //Set volume value. From 0 to 30
  // myDFPlayer.play(1);     //Play the first mp3
  RF_rx.listen();
}
int currentSong=66;
void processCommand(int command)
{
  
  digitalWrite(LED_INDICATOR,HIGH);

  FPSerial.listen();
  while (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read());  //Print the detail message from DFPlayer to handle different errors and states.
  }
  Serial.print("Command received: ");
  Serial.println(command);
  
  if(command==1)
  {
    myDFPlayer.stop();//dừng nhạc
    combination_reset();
    countDown=0;
  }
  else 
  if(command<8){
    countDown=10;
    if(countDown){
    combination_add(command-1);
    int id = combination_getid();
    Serial.print("Combination: ");
    Serial.println(id);
    myDFPlayer.play(id);
    
    }
    else
    {
      Serial.print("Single: ");
      Serial.println(command-1);
      myDFPlayer.play(command-1);
    }

  }
  else if (command==8)
  {
    myDFPlayer.play(64);//chuông
  }
  else if (command==9)
  {
    myDFPlayer.play(65);//còi báo động
  }
  else if(command==10)//bài trước trong list
  {
    // volume+=5;
    // if(volume>30)volume=30;
    // myDFPlayer.nex(volume);
    currentSong--;
    if(currentSong<67)currentSong=88;
    Serial.print("Play song: ");
      Serial.println(currentSong);
    myDFPlayer.play(currentSong);

  }
  else if(command==12)//bài tiếp theo trong list
  {
        currentSong++;
    if(currentSong>88)currentSong=67;
    Serial.print("Play song: ");
      Serial.println(currentSong);
    myDFPlayer.play(currentSong);

  }
  else if(command==11)// quốc ca
  {
Serial.print("Play song: ");
      Serial.println(66);
    myDFPlayer.play(66);

  }
  
  RF_rx.listen();

}
unsigned char RF_byte1 = 0;
unsigned char RF_byte2 = 0;
unsigned char RF_byte3 = 0;
unsigned char RF_byte4 = 0;
int oldtime=0;
void loop() {
  int sec = (millis()/1000);
  if(oldtime!=sec){
    oldtime=sec;
    if(countDown>0){
      
      Serial.println(countDown);
      countDown--;
      if(countDown<=0)combination_reset();
    }
  }
  
  int btn = check_btn();
  if (btn && sdok) {
    processCommand(btn);
    RF_rx.listen();
  }
  RF_rx.listen();
  while (RF_rx.available()) {
    RF_byte4 = RF_byte3;
    RF_byte3 = RF_byte2;
    RF_byte2 = RF_byte1;
    RF_byte1 = RF_rx.read();
    Serial.println(RF_byte1);
    if ((RF_byte3 == 0x04) && (RF_byte4 == 85)) {
      if (RF_byte1 <= 12) {
        processCommand(RF_byte1);
      }
    }
  }
}

void printDetail(uint8_t type, int value) {
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}