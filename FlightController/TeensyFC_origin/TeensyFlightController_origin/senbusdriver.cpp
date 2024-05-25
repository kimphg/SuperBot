#include <cmath>
// #include "usb_serial.h"
#include "senbusdriver.h"
std::vector<String> splitString(String input,char sep)
{
  std::vector<String> tokens;
  if(input.length()<1)return tokens;
  
  int last_sep_pos=0;
  while(1)
  {
    int sep_pos = input.indexOf(sep,last_sep_pos);
    if(sep_pos<0){
      break;
    }
    String token = input.substring(last_sep_pos,sep_pos);
    last_sep_pos = sep_pos+1;
    tokens.push_back(token);
  }
  // DPRINTLN(tokens.size());
  // for(unsigned int i=0;i<tokens.size();i++)
  // {
  //   DPRINTLN(tokens[i]);
  // }
  return tokens;
}
bool SenBusDriver::Input(unsigned char inputByte, float roll,float pitch) {

    bool result =false;
    
    sensBusBuff[sensBusBuffi] = inputByte;
    if(isPrintable (inputByte))
    { 
      // Serial.write(inputByte);
      if (inputString.length() < 100) inputString += (char)inputByte;
      else inputString = "";
      if (inputByte == '#')  //end of command
      {
        
        if (inputString.indexOf("$CAM")>=0) {
          result = processCamera(inputString,roll,pitch);
          
        }
        inputString = "";
      }
    }
    
    sensBusBuffo = inputByte;
    sensBusBuffi++;
    if (sensBusBuffi >= 200) sensBusBuffi = 0;
    return result;
}
bool SenBusDriver::processCamera(String inputStr, float roll,float pitch)
{
    
    bool result =false;
    float tagSize;
    float tag_distance=0;
    std::vector<String> tokens = splitString(inputStr,',');
    if (tokens.size() >= 7) {
      if(tokens[3].equals("TD"))
      {
        int newtagID = tokens[4].toInt();
        float realTagSize = 0;
        if(lastTagID == newtagID){
          tagAngle = tokens[7].toFloat()/10.0;
          if(tagAngle>180.0)tagAngle-=360;
          
          tagID = newtagID;
          tagSize = (tokens[8].toFloat()+tokens[9].toFloat())/2.0;
          float tagDim=1;
          if((tagID==30)&&(tagSize>5))
          {
            realTagSize = 0.12;

          }
          else if((tagID==31)&&(tagSize>5))
          {

            realTagSize = 0.04;
          }
          
          tagX = (50-tokens[5].toFloat());
          tagY = -(50-tokens[6].toFloat());
          result=true;
          float tagWidthAngle = 62.7*(tagSize)/100.0;
          float tagDistance = realTagSize/tan(tagWidthAngle/57.3);
          float alphaRoll = 62.7*tagX/100.0;
          float gammaRoll = roll-alphaRoll;
          dRoll = tagDistance*sin(gammaRoll/57.3);
          float alphaPitch = 62.7*tagY/100.0;
          float gammaPitch = pitch-alphaPitch;
          dPitch = tagDistance*sin(gammaPitch/57.3);
          // Serial.print("!$Camera data:");Serial.print(tagID);Serial.print(" tagDistance: ");Serial.print(tagDistance);Serial.print("   dRoll: ");Serial.print(dRoll);Serial.print("   dPitch: ");Serial.print(dPitch); Serial.print("\n");
        }
        lastTagID = newtagID;
      }
      
      // double roll = 0;
      // double d = 62.7*(tagSize)/100;
     
      // double h = d*(cos(roll - 62.7*(tokens[5].toFloat())));
      // Serial.print("!$Camera data:");Serial.print("d:");Serial.print(d);Serial.print("   h:");Serial.print(h);Serial.print("\n");

    }
    
    return result;
}  