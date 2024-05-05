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
bool SenBusDriver::Input(unsigned char inputByte) {

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
    float tagSize;
    float tag_distance=0;
    std::vector<String> tokens = splitString(inputStr,',');
    if (tokens.size() >= 7) {
      if(tokens[3].equals("TD"))
      {
        int newtagID = tokens[4].toInt();
        
        if(lastTagID == newtagID){
          tagAngle = tokens[7].toFloat()/10.0;
          if(tagAngle>180.0)tagAngle-=360;
          
          tagID = newtagID;
          tagSize = (tokens[8].toFloat()+tokens[9].toFloat())/2.0;
          float tagDim=1;
          if((tagID==31)&&(tagSize>5))
          {
            tag_distance = 6.5/(tagSize) - 0.17;
            tagDim = 0.05;
          }
          else if((tagID==30)&&(tagSize>5))
          {
            tag_distance = 15.0/(tagSize) - 0.0;
            tagDim = 0.14;
          }
          
          tagX = (50-tokens[5].toFloat())*tagDim*tagSize*1.1;
          tagY = -(50-tokens[6].toFloat())*tagDim*tagSize*1.1;
          result=true;
        }
        lastTagID = newtagID;
      }
      Serial.print("!$Camera data:");Serial.print(tagID);Serial.print(" ");Serial.print(tagX);Serial.print(" ");Serial.print(tagY); Serial.print(" ");Serial.print(tagAngle);Serial.print("\n");
    }
    
    return result;
}  