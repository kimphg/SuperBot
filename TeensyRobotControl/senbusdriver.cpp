// #include "usb_serial.h"
#include "senbusdriver.h"
int SenBusDriver::Input(unsigned char inputByte) {
// Serial.write(inputByte);
    int result =0;
    // sensBusBuff[sensBusBuffi] = inputByte;
    if(isPrintable (inputByte))
    { 
      // Serial.print((char)inputByte);
      if (inputString.length() < 200) inputString += (char)inputByte;
      else inputString = "";
      if (inputByte == '\n')  //end of command
      {
        // if(inputString)
        int strlen =inputString.length();
        if(strlen<4)return 0;
        if((strlen>2)&&(inputString.charAt(strlen-2)!='#'))
        {
          return 0;
          // Serial.println("wrong packet:");
          // Serial.println( inputString);
          // Serial.println(inputString.charAt(strlen-2));

        }
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
        float angle = tokens[7].toFloat()/10.0;
        if(angle>360)return 0;
        if(angle<0)return 0;
        float tagx = -(50-tokens[5].toFloat())*1.44;
        float tagy = (50-tokens[6].toFloat())*1.44;
        camtop.setValue(newtagID,tagx,tagy,angle);
        result=4;
        // Serial.print("Camera top:");
        // Serial.print(camtop.stable);Serial.print(",");
        // Serial.print(tagx);Serial.print(",");
        // Serial.print(tagy);Serial.print(",");
        // Serial.println(angle);
        
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
        float angle = tokens[7].toFloat()/10.0;
        if(angle>360)return 0;
        if(angle<0)return 0;
        float tagx = -(50-tokens[5].toFloat())*1.44;
        float tagy = (50-tokens[6].toFloat())*1.44;
        cambot.setValue(newtagID,tagx,tagy,angle);
        result=1;
        // Serial.print("Camera bot:");
        // Serial.print(tagx);Serial.print(",");
        // Serial.print(tagy);Serial.print(",");
        // Serial.println(angle);
        
      }
      
      // DPRINT("!$Camera data:");DPRINTLN(tagAngle);DPRINTLN(tagX);DPRINTLN(tagY); DPRINT("#");DPRINT("@");
    }
    
    return result;
}  