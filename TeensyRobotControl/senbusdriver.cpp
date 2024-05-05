// #include "usb_serial.h"
#include "senbusdriver.h"
bool SenBusDriver::Input(unsigned char inputByte) {

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
        int newtagID = tokens[4].toInt();
        
        if(lastTagID == newtagID){
          tagAngle = tokens[7].toFloat()/10.0;
          if(newtagID==6)tagAngle+=1;
          if(tagAngle>180)tagAngle-=360;
          tagX = -(50-tokens[5].toFloat())*1.44;
          tagY = (50-tokens[6].toFloat())*1.44;
          tagID = newtagID;
          
          result=true;
        }
        
        lastTagID = newtagID;
      }
      DPRINT("!$Camera data:");DPRINTLN(tagAngle);DPRINTLN(tagX);DPRINTLN(tagY); DPRINT("#");DPRINT("@");
    }
    
    return result;
}  