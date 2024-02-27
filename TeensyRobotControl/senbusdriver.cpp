// #include "usb_serial.h"
#include "senbusdriver.h"
bool SenBusDriver::Input(unsigned char inputByte) {
    // if (h7ConnectCount > 0) h7ConnectCount--;
    // camh7data.connectCount = h7ConnectCount;
    bool result =false;
    sensBusBuff[sensBusBuffi] = inputByte;
    if(isPrintable (inputByte))
    { 
      
      if (inputString.length() < 100) inputString += (char)inputByte;
      else inputString = "";
      if (inputByte == '\n')  //end of command
      {
        if (inputString.indexOf("$CAM")>=0) {
          result = processCamera(inputString);
          
        }
        inputString = "";
      }
    }
    sensBusBuffo = inputByte;
    sensBusBuffi++;
    if (sensBusBuffi >= 200) sensBusBuffi = 0;
    return result;
  }
bool SenBusDriver::processCamera(String inputStr)
{
    bool result =false;
    
    std::vector<String> tokens = splitString(inputStr,',');
    if (tokens.size() >= 7) {
      if(tokens[3].equals("TD"))
      {
        int tagID = tokens[4].toInt();
        DPRINT("!$Camera data:");DPRINT(inputStr); DPRINT("#");DPRINT("@");
        if((tagID>=1)&&(tagID<=7)){
          tagAngle = tokens[7].toFloat()/10.0;
          result=true;
        }
      }
    }
    
    return result;
}  