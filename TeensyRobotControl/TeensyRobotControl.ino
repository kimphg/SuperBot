#include <Arduino.h>
#include "common.h"

#include <SPI.h>

#include "mti.h"
#include "motorBLVPWM.h"

//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //
//========================================================================================================================//
IntervalTimer timer_data_input;
IntervalTimer timer_control_loop;
motorBLVPWM motorDriver;
#ifdef TEENSY41
//networking configurations
// #include <NativeEthernet.h>         // for Teensy 4.1
// #include <NativeEthernetUdp.h>
// byte mac[] = { 0x04, 0xE9, 0xE5, 0x0B, 0xFC, 0xD1 };    // com 6      Teensy
// IPAddress myIP(192,  168, 0, 16);                        // BCP21 = (160,  48, 199, 16);                    
// IPAddress destinationIP(192,  168, 0, 4);    
// unsigned int destinationPort = 31000;                         // Inova receiver Address
// unsigned int myPort          = 32501;                         // BCP21 orig. Port

// EthernetUDP Udp;  
// void SendToPC(uint8_t* data, int len)
// {
//   Udp.beginPacket(destinationIP, destinationPort);
//   Udp.write(data, len);
//   Udp.endPacket();
// }
// void SendToPC(uint8_t* data)
// {
//   int len = strlen(data);
//   Udp.beginPacket(destinationIP, destinationPort);
//   Udp.write(data, len);
//   Udp.endPacket();
// }
#endif
//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //
//========================================================================================================================//

//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
unsigned long channel_1_fs = 1000; //thro
unsigned long channel_2_fs = 1500; //ail
unsigned long channel_3_fs = 1500; //elev
unsigned long channel_4_fs = 1500; //rudd
unsigned long channel_5_fs = 1000; //gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 1000; //aux1

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.04;  //Madgwick filter parameter
float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
float B_mag = 1.0;        //Magnetometer LP filter parameter

//Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
float MagErrorX = 0.0;
float MagErrorY = 0.0;
float MagErrorZ = 0.0;
float MagScaleX = 1.0;
float MagScaleY = 1.0;
float MagScaleZ = 1.0;

//Controller parameters (take note of defaults before modifying!):
float i_limit = 3.0;     //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 20.0;     //Max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
float maxPitch = 20.0;    //Max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
float maxYaw = 160.0;     //Max yaw rate in deg/sec

float Kp_roll_angle = 0.1;    //Roll P-gain - angle mode
float Ki_roll_angle = 0.1;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (if using controlANGLE2(), set to 0.0)
float B_loop_roll = 0.9;      //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.0;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.00;  //Pitch D-gain - angle mode (if using controlANGLE2(), set to 0.0)
float B_loop_pitch = 0.9;     //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)





//========================================================================================================================//
//                                                     DECLARE PINS                                                       //
//========================================================================================================================//

// #include "common.h"

//========================================================================================================================//



//DECLARE GLOBAL VARIABLES

//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;
//Radio comm:
 int channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
 int channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

#if defined USE_SBUS_RX
SBUS sbus(Serial5);
uint16_t sbusChannels[16];
bool sbusFailSafe;
bool sbusLostFrame;
#endif

//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
float q0 = 1.0f; //initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Normalized desired state:
float spd_des=0, roll_des, pitch_des, yaw_des=0, pos_des=0;
int motorActive = 0;
float pos_curr=0;
float roll_passthru, pitch_passthru, yaw_passthru;

//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;


//Mixer

int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;

IMU_driver imu;

//========================================================================================================================//
//                                                      VOID SETUP                                                        //
//========================================================================================================================//
void commandMotorsOneShot() ;
unsigned char sensBusBuff[200];
int sensBusBuffi=0;
unsigned char sensBusBuffo;
int poscamConnect = 0;
int h7ConnectCount=0;
void readSensBus()
{
  if(h7ConnectCount>0)h7ConnectCount--;
  camh7data.connectCount = h7ConnectCount;
  while(RS485_SENS.available())
  {
    
    unsigned char inputByte = RS485_SENS.read();
    sensBusBuff[sensBusBuffi] =inputByte;
    if(sensBusBuffo==0xAA)if(inputByte==0x55)
    {
      // DEBUG_TELEMETRY.println(sensBusBuffi);
      if(sensBusBuffi==12)if(sensBusBuff[0]=0x01)
      {
        if(sensBusBuff[1]=0x11){
          int angleTag = sensBusBuff[7]*256+sensBusBuff[8];
          camh7data.x = sensBusBuff[5]-127;
          camh7data.y = 137- sensBusBuff[6];
          camh7data.angle = angleTag/10.0;
          h7ConnectCount=200;
          // DEBUG_TELEMETRY.print(sensBusBuff[5]);DEBUG_TELEMETRY.print(" ");
          // DEBUG_TELEMETRY.print(sensBusBuff[6]);DEBUG_TELEMETRY.print(" ");
          // DEBUG_TELEMETRY.println(angleTag); DEBUG_TELEMETRY.print(" ");
          // DEBUG_TELEMETRY.print(sensBusBuff[6]);DEBUG_TELEMETRY.print("\r\n");
        }
      }
      sensBusBuffi=0;
    }
    sensBusBuffo= inputByte;
    sensBusBuffi++;
    if(sensBusBuffi>=200)sensBusBuffi=0;
  }
}
int currState=0;
int stateStepID = 0;
void gotoState(int state)
{
    currState = state;
    stateStepID = 0;
    motorDriver.resetPosition();
    DEBUG_TELEMETRY.print("gotoState:");
    DEBUG_TELEMETRY.println(state);
}
void loopState()
{
  
  if(currState==4)//Tag searching sequence
  {
    // DEBUG_TELEMETRY.print("currState==4:");
    if(stateStepID==0)// find tag drection stage
    {
      // DEBUG_TELEMETRY.println(camh7data.connectCount);
      if(camh7data.connectCount!=0)
      {
        float distance = sqrt(camh7data.y*camh7data.y+camh7data.x*camh7data.x);
        DEBUG_TELEMETRY.print("distance:");
        DEBUG_TELEMETRY.println(distance);
        // if(distance<15)stateStepID=3;
        float tagBearing=0;
        if(camh7data.y>0)
          tagBearing = atan(camh7data.x/camh7data.y)*57.2958;
        else if(camh7data.y<0)
          tagBearing = atan(camh7data.x/camh7data.y)*57.2958+180;
        else if(camh7data.x<0)tagBearing=-90;
        else tagBearing=90;
        yaw_des = yaw_IMU+tagBearing;
        if(yaw_des>180.0)yaw_des-=360.0;
        DEBUG_TELEMETRY.print(yaw_des);
        // motorDriver.SetControlValue(yaw_des,0);
        stateStepID=1;
        DEBUG_TELEMETRY.print("Step up:");
        DEBUG_TELEMETRY.println(stateStepID);
        
      }
      else{
        DEBUG_TELEMETRY.println("Tag not detected ");
      }
    }
    if(stateStepID==1)// rotate to tag drection stage
    {
      float distance = sqrt(camh7data.y*camh7data.y+camh7data.x*camh7data.x);
        DEBUG_TELEMETRY.print("distance:");
        DEBUG_TELEMETRY.println(distance);
      DEBUG_TELEMETRY.print("Step 1:");
        DEBUG_TELEMETRY.println(yaw_des-yaw_IMU);
      if(abs(yaw_des-yaw_IMU)<0.5)
          if(motorDriver.getRotSpeed()<0.05)
            if(camh7data.connectCount!=0)
      {
        stateStepID=2;
        DEBUG_TELEMETRY.print("Step up:");
        DEBUG_TELEMETRY.println(stateStepID);
          }
          
    }
    if(stateStepID==2)// move to tag position stage
    {
      if(abs(yaw_des-yaw_IMU)<0.5)
          if(motorDriver.getRotSpeed()<0.05)
          if(camh7data.connectCount!=0)
        {
          
          // motorDriver.SetControlValue(yaw_IMU,camh7data.y);
          // stateStepID=2;
          DEBUG_TELEMETRY.print(camh7data.y);
          DEBUG_TELEMETRY.print("Step up");
          DEBUG_TELEMETRY.println(stateStepID);
          
        }
        else{
          DEBUG_TELEMETRY.println("Tag not detected ");
        }
    }
    
  }

}
void setup() {
  Serial.begin(115200); //usb serial
  DEBUG_TELEMETRY.begin(57600); //telemetry serial
  RS485_IMU.begin(921600);
  RS485_SENS.begin(1000000);
  delay(200);
  imu.IMU_init(RS485_IMU);
  Serial.println("start");

  //Initialize all pins
  // pinMode(13, OUTPUT); //pin 13 LED blinker on board, do not modify
  if(imu.getIsConnected())
  {
    Serial.println("IMU connect OK");

  }else {
    Serial.println("IMU connect failed");
    indicateErrorLed(3);
  }
  Serial.flush();
  //Initialize radio communication
  radioSetup();
  timer_data_input.begin(inputDataUpdate, 300);//
  timer_control_loop.begin(controlUpdate, int(DT_CONTROL*1000000));
  Serial.println("IMU timer started");
  //Set radio channels to default (safe) values before entering main loop
  // channel_1_pwm = channel_1_fs;
  // channel_2_pwm = channel_2_fs;
  // channel_3_pwm = channel_3_fs;
  // channel_4_pwm = channel_4_fs;
  // channel_5_pwm = channel_5_fs;
  // channel_6_pwm = channel_6_fs;
  // indicateErrorLed(0);
  prev_time = 0;
  Serial.println("Setup done, enter main loop");
}

int debugID=0;
#define COMMAND_LEN_MAX 200
uint8_t udpBuff[300];
int commandBuffIndex = 0;
//========================================================================================================================//
//                                                       MAIN LOOP                                                        //
//========================================================================================================================//
void printBatVoltage()
{

  float valueInput = analogRead(BAT_VOL_SENS);
  valueInput *= 0.5615;
  Serial.print("Bat:");
  Serial.println(valueInput);

}
bool manualMode = false;
int loopRatePeriodUS=50000;
int loopCountActive=0;
void loop() {
  current_time = micros();          //looprate limiter
  dt = (current_time - prev_time);  //
  if(dt<loopRatePeriodUS)return;    //
  if(dt>loopRatePeriodUS+10){
    Serial.print("loop too slow error:");
    Serial.println(dt);
  }
  
  prev_time = current_time;         //
//  if(imu.noMotionCount>100)//200ms
//  digitalWrite(13,HIGH);
//  else  digitalWrite(13,LOW);
 
  // loopBlink(); //indicate we are in main loop with short blink every 1.5 seconds
  if (current_time - print_counter > 100000) {
    print_counter = micros();
    debugID++;
    if(debugID>10)debugID=0;
    if(debugID==1)
    {
      
      // sprintf(udpBuff,"Radio data:\t%d\t%d\t%d\t%d\t%d",channel_1_pwm,channel_2_pwm,channel_3_pwm,channel_4_pwm,channel_5_pwm);
      // Serial.printf(udpBuff);
      // SendToPC(udpBuff);
      
    }
    
  //  DEBUG_TELEMETRY.print(dt);
  //  DEBUG_TELEMETRY.print(" \n");
    //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
    // printRadioData();     //radio pwm values (expected: 1000 to 2000)
//    printDesiredState();  //prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
      //  printGyroData();      //prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
    //      printAccelData();     //prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
    //    printMagData();       //prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
        //  printRollPitchYaw();  //prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
    //      printPIDoutput();     //prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
    // printMotorCommands(); //prints the values being written to the motors (expected: 120 to 250)
    //    printBatVoltage();
    //    printServoCommands(); //prints the values being written to the servos (expected: 0 to 180)
    //printLoopRate();      //prints the time between loops in microseconds (expected: microseconds between loop iterations)
  }
  
  //Get vehicle state
  // getIMUdata(); //pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  //Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); //updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)
  updateSensors();
  // updateCommandBus();
  // DEBUG_TELEMETRY.print(imu.gyroZBiasCompensation*100000);
  // DEBUG_TELEMETRY.print(" ");
  // DEBUG_TELEMETRY.println(yaw_IMU);
  // Madgwick6DOF(GyroX, GyroY, GyroZ, AccX, AccY, AccZ,  dt);
  //Compute desired state
  // Serial.println(dt);
  if(1){
    getCommandsRadio(); 
        //Yaw, stablize on rate from GyroZ
    // error_yaw = yaw_des - yaw_IMU;
    // if(error_yaw>180)error_yaw-=360;
    // if(error_yaw<-180)error_yaw+=360;
    // integral_yaw = integral_yaw_prev + error_yaw * dt/1000000.0;
    // if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    //   integral_yaw = 0;
    // }
    // integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
    // derivative_yaw = (error_yaw - error_yaw_prev) / dt*10000000.0;
    // yaw_PID = .3 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw); //scaled by .01 to bring within -1 to 1 range
    // error_yaw_prev = error_yaw;
    // Serial.println((channel_4_pwm-1500)/5000.0);
    //send command to motors
    // Serial.println(yaw_IMU);
    
    if (channel_5_pwm < 1500)
    {
      // imu.resetYaw();
      // motorDriver.robotPosition=0;
    }
    // motorDriver.isActive=motorActive;
    if (motorDriver.isActive==false) {
      
      motorDriver.resetPosition();
      imu.resetYaw();
      loopCountActive=0;
      // Serial.println("inactive");
    }
    else{
      loopCountActive++;
      loopState();
      // motorDriver.isActive=true;
      // int rotation=(channel_4_pwm-1500);
      // float curTime=millis()/1000.0;
      // float speed = (channel_3_pwm-1500.0)/2000.0;
      // if (speed<0)speed=0;
      //
      // motorDriver.SetControlValue(yaw_des,pos_des);
      // Serial.println(loopCountActive);

      // if(loopCountActive<50)      motorDriver.SetControlValue(0,0);
      // else if(loopCountActive<300)motorDriver.SetControlValue(0,2000);
      // else if(loopCountActive<400)motorDriver.SetControlValue(90,2000);
      // else if(loopCountActive<550)motorDriver.SetControlValue(90,3000);
      // else if(loopCountActive<650)motorDriver.SetControlValue(180,3000);
      // else if(loopCountActive<900)motorDriver.SetControlValue(180,5000);
      // else if(loopCountActive<1000)motorDriver.SetControlValue(270,5000);
      // else if(loopCountActive<1150)motorDriver.SetControlValue(270,6000);
      // else if(loopCountActive<1000000)motorDriver.SetControlValue(0,6000);
      // else if(loopCountActive<250)motorDriver.SetControlValue(0,0);
      // else if(loopCountActive<220)motorDriver.SetControlValue(-180,0);
      // else if(loopCountActive<300)motorDriver.SetControlValue(90,0);
      // else if(loopCountActive<370)motorDriver.SetControlValue(-180,0);
      // else if(loopCountActive<500)motorDriver.SetControlValue(0,2000);

      // if(curTime>5&&curTime<8)motorDriver.SetControlValue(0.2,0);
      // else if(curTime>8&&curTime<10)motorDriver.SetControlValue(0,45);
      // else if(curTime>10&&curTime<12)motorDriver.SetControlValue(0,90);
      // else if(curTime>12&&curTime<14)      motorDriver.SetControlValue(0,135);
      // else if(curTime>14&&curTime<16)      motorDriver.SetControlValue(0,180);
      // else if(curTime>16&&curTime<19)motorDriver.SetControlValue(0.20,180);
      // else motorDriver.isActive=false;
    }
    // commandMotorsBLVM(); 
  }
  // failSafe(); //prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  //Regulate loop rate
  // loopRate(100); //do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}



//========================================================================================================================//
//                                                      FUNCTIONS                                                         //
//========================================================================================================================//

void IMUinit() {
  //DESCRIPTION: Initialize IMU
  /*
     Don't worry about how this works
  */
#if defined USE_MPU6050_I2C
  Wire.begin();
  Wire.setClock(600000); //Note this is 2.5 times the spec sheet 400 kHz max...

  mpu6050.initialize();

  while  (mpu6050.testConnection() == false) {
    Serial.println("MPU6050 initialization unsuccessful");
    Serial.println("Check MPU6050 wiring or try cycling power");
    delay(1000);
    mpu6050.initialize();
  }

  //From the reset state all registers should be 0x00, so we should be at
  //max sample rate with digital low pass filter(s) off.  All we need to
  //do is set the desired fullscale ranges
  mpu6050.setFullScaleGyroRange(GYRO_SCALE);
  mpu6050.setFullScaleAccelRange(ACCEL_SCALE);

#elif defined USE_MPU9250_SPI
  int status = mpu9250.begin();

  if (status < 0) {
    Serial.println("MPU9250 initialization unsuccessful");
    Serial.println("Check MPU9250 wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  //From the reset state all registers should be 0x00, so we should be at
  //max sample rate with digital low pass filter(s) off.  All we need to
  //do is set the desired fullscale ranges
  mpu9250.setGyroRange(GYRO_SCALE);
  mpu9250.setAccelRange(ACCEL_SCALE);
  mpu9250.setMagCalX(MagErrorX, MagScaleX);
  mpu9250.setMagCalY(MagErrorY, MagScaleY);
  mpu9250.setMagCalZ(MagErrorZ, MagScaleZ);
  mpu9250.setSrd(0); //sets gyro and accel read to 1khz, magnetometer read to 100hz
#endif
}
void updateSensors()
{
  
  IMUData measure = imu.getMeasurement();
  AccX = measure.accY;
  AccY = measure.accY;
  AccZ = measure.accZ;
  GyroX = measure.gyroX*57.2958;
  GyroY = measure.gyroY*57.2958;
  GyroZ = measure.gyroZ*57.2958;
  // yaw_IMU = measure.gyroyaw*57.2958;
  yaw_IMU = measure.gyroyaw;
  // Serial.print(yaw_IMU);
  // Serial.print(" ");
  // Serial.print(measure.yaw-170.3);
  // Serial.print(" ");
  // Serial.println(imu.yawCalcMode-100);
}
String commandString;
void updateCommandBus()
{
  while(DEBUG_TELEMETRY.available())
  {
    uint8_t bytein = DEBUG_TELEMETRY.read();
    DEBUG_TELEMETRY.print(bytein);
    if(commandString.length()<COMMAND_LEN_MAX)commandString+=(char)bytein;
    if(bytein=='\n')//end of command
    {
      int dataLen = commandString.length();
      // DEBUG_TELEMETRY.println(dataLen);
    
      if(commandString.startsWith("yaw="))//angle set command
      {
        
        // float angle
        //   DEBUG_TELEMETRY.print(imu.gyroZBiasCompensation*100000);
        yaw_des = commandString.substring(4,dataLen-1).toFloat();
        DEBUG_TELEMETRY.print("yaw_des set:");
        DEBUG_TELEMETRY.println(yaw_des);
        //   DEBUG_TELEMETRY.println(yaw_IMU);
      }
      else if(commandString.startsWith("pos="))//angle set command
      {
        
        // float angle
        //   DEBUG_TELEMETRY.print(imu.gyroZBiasCompensation*100000);
        pos_des = commandString.substring(4,dataLen-1).toFloat();
        DEBUG_TELEMETRY.print("pos_des set:");
        DEBUG_TELEMETRY.println(pos_des);
        //   DEBUG_TELEMETRY.println(yaw_IMU);
      }
      else if(commandString.startsWith("resetyaw"))//angle set command
      {
        imu.resetYaw();
      }
      else if(commandString.startsWith("stt="))//active set command
      {
        
        // float angle
        //   DEBUG_TELEMETRY.print(imu.gyroZBiasCompensation*100000);
        int newstat = commandString.substring(4,dataLen-1).toFloat();
        gotoState(newstat);
        //   DEBUG_TELEMETRY.println(yaw_IMU);
      }
      commandString = "";
      
      
    }
      
  }
}

void getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
  /*
     Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ.
     These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order
     low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
     off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
     the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally,
     the constant errors found in calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
  */
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ;

#if defined USE_MPU6050_I2C
  mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
#elif defined USE_MPU9250_SPI
  mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
#endif

  //Accelerometer
  AccX = AcX / ACCEL_SCALE_FACTOR; //G's
  AccY = AcY / ACCEL_SCALE_FACTOR;
  AccZ = AcZ / ACCEL_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  //LP filter accelerometer data
  AccX = (1.0 - B_accel) * AccX_prev + B_accel * AccX;
  AccY = (1.0 - B_accel) * AccY_prev + B_accel * AccY;
  AccZ = (1.0 - B_accel) * AccZ_prev + B_accel * AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  //Gyro
  GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
  GyroY = GyY / GYRO_SCALE_FACTOR;
  GyroZ = GyZ / GYRO_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  //LP filter gyro data
  GyroX = (1.0 - B_gyro) * GyroX_prev + B_gyro * GyroX;
  GyroY = (1.0 - B_gyro) * GyroY_prev + B_gyro * GyroY;
  GyroZ = (1.0 - B_gyro) * GyroZ_prev + B_gyro * GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;

  //Magnetometer
  MagX = MgX / 6.0; //uT
  MagY = MgY / 6.0;
  MagZ = MgZ / 6.0;
  //Correct the outputs with the calculated error values
  MagX = (MagX - MagErrorX) * MagScaleX;
  MagY = (MagY - MagErrorY) * MagScaleY;
  MagZ = (MagZ - MagErrorZ) * MagScaleZ;
  //LP filter magnetometer data
  MagX = (1.0 - B_mag) * MagX_prev + B_mag * MagX;
  MagY = (1.0 - B_mag) * MagY_prev + B_mag * MagY;
  MagZ = (1.0 - B_mag) * MagZ_prev + B_mag * MagZ;
  MagX_prev = MagX;
  MagY_prev = MagY;
  MagZ_prev = MagZ;
}
void commandMotorPWM()
{

}
void commandMotorsPWM()
{

}

void calculate_IMU_error() {
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  /*
     Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and
     accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
     measurement.
  */
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ;

  //Read IMU values 12000 times
  int c = 0;
  while (c < 18000) {
#if defined USE_MPU6050_I2C
    mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
#elif defined USE_MPU9250_SPI
    mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
#endif

    AccX  = AcX / ACCEL_SCALE_FACTOR;
    AccY  = AcY / ACCEL_SCALE_FACTOR;
    AccZ  = AcZ / ACCEL_SCALE_FACTOR;
    GyroX = GyX / GYRO_SCALE_FACTOR;
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;

    //Sum all readings
    AccErrorX  = AccErrorX + AccX;
    AccErrorY  = AccErrorY + AccY;
    AccErrorZ  = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;
}

void calibrateAttitude() {
  //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
  //Assuming vehicle is powered up on level surface!
  /*
     This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
     to boot.
  */
  //Warm up IMU and madgwick filter in simulated main loop
  for (int i = 0; i <= 15000; i++) {
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    getIMUdata();
    
    Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt);
    loopRate(2000); //do not exceed 2000Hz
  }
}

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
  /*
     This function fuses the accelerometer gyro, and magnetometer readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, and MagZ for attitude estimation.
     Don't worry about the math. There is a tunable parameter B_madgwick in the user specified variable section which basically
     adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower
     beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,
     pitch_IMU, and yaw_IMU variables which are in degrees. If magnetometer data is not available, this function calls Madgwick6DOF() instead.
  */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float mholder;

  //use 6DOF algorithm if MPU6050 is being used
#if defined USE_MPU6050_I2C
  Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
  return;
#endif

  //Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
  }

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    //Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    //Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //compute angles - NWU
  roll_IMU = atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 57.29577951; //degrees
  pitch_IMU = -asin(-2.0f * (q1 * q3 - q0 * q2)) * 57.29577951; //degrees
  yaw_IMU = -atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * 57.29577951; //degrees
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
     See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
     available (for example when using the recommended MPU6050 IMU for the default setup).
  */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 , _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //compute angles
  roll_IMU = atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 57.29577951; //degrees
  pitch_IMU = -asin(-2.0f * (q1 * q3 - q0 * q2)) * 57.29577951; //degrees
  yaw_IMU = -atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * 57.29577951; //degrees
}

void getDesState() {

  spd_des = (channel_3_pwm - 1000.0) / 1000.0; //between 0 and 1
  if(spd_des<0.1)spd_des=0;
  roll_des = (channel_1_pwm - 1500.0) / 500.0; //between -1 and 1
  pitch_des = (channel_2_pwm - 1500.0) / 500.0; //between -1 and 1
  yaw_des = -(channel_4_pwm - 1500.0) / 500.0; //between -1 and 1
  //Constrain within normalized bounds
  spd_des = constrain(spd_des, 0.0, 1.0); //between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0) * maxRoll; //between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0) * maxPitch; //between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0) * maxYaw; //between -maxYaw and +maxYaw

  roll_passthru = roll_des / (2 * maxRoll);
  pitch_passthru = pitch_des / (2 * maxPitch);
  yaw_passthru = yaw_des / (2 * maxYaw);
}

void controlRATE() {
  //DESCRIPTION: Computes control commands based on state error (rate)
  /*
     See explanation for controlANGLE(). Everything is the same here except the error is now the desired rate - raw gyro reading.
  */
  //Roll
  error_roll = roll_des - GyroX;
  integral_roll = integral_roll_prev + error_roll * dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev) / dt;
  roll_PID = .01 * (Kp_roll_rate * error_roll + Ki_roll_rate * integral_roll + Kd_roll_rate * derivative_roll); //scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev) / dt;
  pitch_PID = .01 * (Kp_pitch_rate * error_pitch + Ki_pitch_rate * integral_pitch + Kd_pitch_rate * derivative_pitch); //scaled by .01 to bring within -1 to 1 range

  
  //Update roll variables
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  GyroX_prev = GyroX;
  //Update pitch variables
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  GyroY_prev = GyroY;
  //Update yaw variables
  // error_yaw_prev = error_yaw;
  // integral_yaw_prev = integral_yaw;
}

void controlMixer() {

  

}


void getCommandsRadio() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
     Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of
     the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which
     is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the alues are pulled from the SBUS library directly.
     The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise.
  */

#if defined USE_PPM_RX || defined USE_PWM_RX
  channel_1_pwm = getRadioPWM(1);
  channel_2_pwm = getRadioPWM(2);
  channel_3_pwm = getRadioPWM(3);
  channel_4_pwm = getRadioPWM(4);
  channel_5_pwm = getRadioPWM(5);
  channel_6_pwm = getRadioPWM(6);

#elif defined USE_SBUS_RX
  if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame))
  {
    //sBus scaling below is for Taranis-Plus and X4R-SB
    float scale = 0.615;
    float bias  = 895.0;
    channel_1_pwm = sbusChannels[0] * scale + bias;
    channel_2_pwm = sbusChannels[1] * scale + bias;
    channel_3_pwm = sbusChannels[2] * scale + bias;
    channel_4_pwm = sbusChannels[3] * scale + bias;
    channel_5_pwm = sbusChannels[4] * scale + bias;
    channel_6_pwm = sbusChannels[5] * scale + bias;
  }
#endif

  //Low-pass the critical commands and update previous values
  float b = 0.2; //lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b) * channel_1_pwm_prev + b * channel_1_pwm;
  channel_2_pwm = (1.0 - b) * channel_2_pwm_prev + b * channel_2_pwm;
  channel_3_pwm = (1.0 - b) * channel_3_pwm_prev + b * channel_3_pwm;
  channel_4_pwm = (1.0 - b) * channel_4_pwm_prev + b * channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void failSafe() {
  //DESCRIPTION: If radio gives garbage values, set all commands to default values
  /*
     Radio connection failsafe used to check if the getCommandsRadio() function is returning acceptable pwm values. If any of
     the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
     connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands
     channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting
     your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
  */
  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;

  //Triggers for failure criteria
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal) check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal) check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal) check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal) check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal) check5 = 1;
  if (channel_6_pwm > maxVal || channel_6_pwm < minVal) check6 = 1;

  //If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
  }
}


float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq) {
  float diffParam = (param_max - param_min) / (fadeTime * loopFreq); //difference to add or subtract from param for each loop iteration for desired fadeTime

  if (state == 1) { //maximum param bound desired, increase param by diffParam for each loop iteration
    param = param + diffParam;
  }
  else if (state == 0) { //minimum param bound desired, decrease param by diffParam for each loop iteration
    param = param - diffParam;
  }

  param = constrain(param, param_min, param_max); //constrain param within max bounds

  return param;
}

float switchRollYaw(int reverseRoll, int reverseYaw) {

  float switch_holder;

  switch_holder = yaw_des;
  yaw_des = reverseYaw * roll_des;
  roll_des = reverseRoll * switch_holder;
}


void throttleCut() {
  //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
     Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is
     minimum for oneshot125 protocol, 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function
     called before commandMotorsPWM() is called so that the last thing checked is if the user is giving permission to command
     the motors to anything other than minimum value. Safety first.
  */
  // if (channel_5_pwm < 1500) {
  //   motorLset=0;
  //   motorRset=0;
  // }
}

void calibrateMagnetometer() {
#if defined USE_MPU9250_SPI
  float success;
  Serial.println("Beginning magnetometer calibration in");
  Serial.println("3...");
  delay(1000);
  Serial.println("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);
  Serial.println("Rotate the IMU about all axes until complete.");
  Serial.println(" ");
  success = mpu9250.calibrateMag();
  if (success) {
    Serial.println("Calibration Successful!");
    Serial.println("Please comment out the calibrateMagnetometer() function and copy these values into the code:");
    Serial.print("float MagErrorX = ");
    Serial.print(mpu9250.getMagBiasX_uT());
    Serial.println(";");
    Serial.print("float MagErrorY = ");
    Serial.print(mpu9250.getMagBiasY_uT());
    Serial.println(";");
    Serial.print("float MagErrorZ = ");
    Serial.print(mpu9250.getMagBiasZ_uT());
    Serial.println(";");
    Serial.print("float MagScaleX = ");
    Serial.print(mpu9250.getMagScaleFactorX());
    Serial.println(";");
    Serial.print("float MagScaleY = ");
    Serial.print(mpu9250.getMagScaleFactorY());
    Serial.println(";");
    Serial.print("float MagScaleZ = ");
    Serial.print(mpu9250.getMagScaleFactorZ());
    Serial.println(";");
    Serial.println(" ");
    Serial.println("If you are having trouble with your attitude estimate at a new flying location, repeat this process as needed.");
  }
  else {
    Serial.println("Calibration Unsuccessful. Please reset the board and try again.");
  }

  while (1); //halt code so it won't enter main loop until this function commented out
#endif
  Serial.println("Error: MPU9250 not selected. Cannot calibrate non-existent magnetometer.");
  while (1); //halt code so it won't enter main loop until this function commented out
}

void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
     It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
     background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until
     the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to
     be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
     and remain above 2kHz, without needing to retune all of our filtering parameters.
  */
  float invFreq = 1.0 / freq * 1000000.0;
  unsigned long checker = micros();

  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void loopBlink() {
  //DESCRIPTION: Blink LED on board to indicate main loop is running
  /*
     It looks cool.
  */
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    // digitalWrite(13, blinkAlternate); //pin 13 is built in LED

    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
    }
    else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
    }
  }
}



void printRadioData() {

  DEBUG_TELEMETRY.print(F(" CH1: "));
  DEBUG_TELEMETRY.print(channel_1_pwm);
  DEBUG_TELEMETRY.print(F(" CH2: "));
  DEBUG_TELEMETRY.print(channel_2_pwm);
  DEBUG_TELEMETRY.print(F(" CH3: "));
  DEBUG_TELEMETRY.print(channel_3_pwm);
  DEBUG_TELEMETRY.print(F(" CH4: "));
  DEBUG_TELEMETRY.print(channel_4_pwm);
  DEBUG_TELEMETRY.print(F(" CH5: "));
  DEBUG_TELEMETRY.print(channel_5_pwm);
  DEBUG_TELEMETRY.print(F(" CH6: "));
  DEBUG_TELEMETRY.println(channel_6_pwm);

}

void printDesiredState() {

  DEBUG_TELEMETRY.print(F("thro_des: "));
  DEBUG_TELEMETRY.print(spd_des);
  DEBUG_TELEMETRY.print(F(" roll_des: "));
  DEBUG_TELEMETRY.print(roll_des);
  DEBUG_TELEMETRY.print(F(" pitch_des: "));
  DEBUG_TELEMETRY.print(pitch_des);
  DEBUG_TELEMETRY.print(F(" yaw_des: "));
  DEBUG_TELEMETRY.println(yaw_des);

}

void printGyroData() {

  Serial.print(F("GyroX: "));
  Serial.print(GyroX);
  Serial.print(F(" GyroY: "));
  Serial.print(GyroY);
  Serial.print(F(" GyroZ: "));
  Serial.println(GyroZ);

}

void printAccelData() {

  Serial.print(F("AccX: "));
  Serial.print(AccX);
  Serial.print(F(" AccY: "));
  Serial.print(AccY);
  Serial.print(F(" AccZ: "));
  Serial.println(AccZ);

}

void printMagData() {

  Serial.print(F("MagX: "));
  Serial.print(MagX);
  Serial.print(F(" MagY: "));
  Serial.print(MagY);
  Serial.print(F(" MagZ: "));
  Serial.println(MagZ);

}

void printRollPitchYaw() {

  Serial.print(F("roll: "));
  Serial.print(roll_IMU);
  Serial.print(F(" pitch: "));
  Serial.print(pitch_IMU);
  Serial.print(F(" yaw: "));
  Serial.print(yaw_IMU);
  Serial.print(F(" gyroZBiasCompensation: "));
  Serial.println(imu.gyroZBiasCompensation*1000);

}

void printPIDoutput() {

  // Serial.print(F("roll_PID: "));
  // Serial.print(roll_PID);
  // Serial.print(F(" pitch_PID: "));
  // Serial.print(pitch_PID);
  // Serial.print(F(" yaw_PID: "));
  // Serial.println(yaw_PID);

}

void printMotorCommands() {

  DEBUG_TELEMETRY.print(F("  yaw_des: "));
  DEBUG_TELEMETRY.print(yaw_des);
  DEBUG_TELEMETRY.print(F("  yaw_IMU: "));
  DEBUG_TELEMETRY.print(yaw_IMU);
  // DEBUG_TELEMETRY.print(F("motor left: "));
  // DEBUG_TELEMETRY.print(motorLset);
  // DEBUG_TELEMETRY.print(F(" motor right: "));
  // DEBUG_TELEMETRY.println(motorRset);
  

}

void printServoCommands() {
  Serial.print(F("s1_command: "));
  Serial.print(s1_command_PWM);
  Serial.print(F(" s2_command: "));
  Serial.print(s2_command_PWM);
  Serial.print(F(" s3_command: "));
  Serial.print(s3_command_PWM);
  Serial.print(F(" s4_command: "));
  Serial.print(s4_command_PWM);
  Serial.print(F(" s5_command: "));
  Serial.print(s5_command_PWM);
  Serial.print(F(" s6_command: "));
  Serial.print(s6_command_PWM);
  Serial.print(F(" s7_command: "));
  Serial.println(s7_command_PWM);
}
void printLoopRate() {
  Serial.print(F("dt = "));
  Serial.println(dt * 1000000.0);
}
static void inputDataUpdate()
{
  imu.updateData();//read IMU
  updateCommandBus();//read Serial Commands
  readSensBus();
}
static void controlUpdate()
{
  motorDriver.update(yaw_IMU);
}
