// #include "usb_serial.h"
#include "senbusdriver.h"
int SenBusDriver::Input(unsigned char inputByte) {
Serial.write(inputByte);
    int result =0;
    // sensBusBuff[sensBusBuffi] = inputByte;
    if(isPrintable (inputByte))
    { 
      // Serial.print((char)inputByte);
      if (inputString.length() < 100) inputString += (char)inputByte;
      else inputString = "";
      if (inputByte == '\n')  //end of command
      {
        if (inputString.indexOf("$CAM1,")>=0) {
          result = processCamera(inputString);
        }
        else if (inputString.indexOf("$CAM2,")>=0) {
          result = processCameraTop(inputString);
        }
        else if (inputString.indexOf("$COM,")>=0) {
          result = processCommand(inputString);
        }
        else if (inputString.indexOf("$FRB,")>=0) {
          result = processFrontBoard(inputString);
        }
        inputString = "";
      }
    }
    // sensBusBuffo = inputByte;
    // sensBusBuffi++;
    // if (sensBusBuffi >= 200) sensBusBuffi = 0;
    return result;
}
int SenBusDriver::processCommand(String command)
{
  commandBuff ="";
  commandBuff.append(command);
  // Serial.print(command);
  return 2;
  
}
int SenBusDriver::processFrontBoard(String inputStr)
{
    int result =0;
    
    std::vector<String> tokens = splitString(inputStr,',');
    if (tokens.size() >= 2) {
      int warning_level = tokens[1].toInt();
      if(warning_level>0)
      if(warning_level<4){
        if(fb_warning_level == warning_level) warning_repeated++;
        else 
        {
          fb_warning_level = warning_level;
          warning_repeated=0;
        }
        result = 3;
      }
    }
    
    return result;
} 
int SenBusDriver::processCameraTop(String inputStr)
{
  int result =0;
  
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
          result=2;
          
        }
        else result=0;
        lastTagID = newtagID;
        
      }
    }
  return result ;
    
}
int SenBusDriver::processCamera(String inputStr)
{
    int result =0;
    // Serial.println("Camera bot");
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
          result=1;
          
        }
        else result=0;
        lastTagID = newtagID;
        
      }
      
      // DPRINT("!$Camera data:");DPRINTLN(tagAngle);DPRINTLN(tagX);DPRINTLN(tagY); DPRINT("#");DPRINT("@");
    }
    
    return result;
}  