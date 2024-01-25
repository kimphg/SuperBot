#include <Arduino.h>
#include "common.h"

#include <SPI.h>

#include "cRobotFSM.h"
// #include "motorBLVPWM.h"

//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //
//========================================================================================================================//
IntervalTimer timer_data_input;
IntervalTimer timer_control_loop;
// motorBLVPWM motorDriver;
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
unsigned long channel_1_fs = 1000;  //thro
unsigned long channel_2_fs = 1500;  //ail
unsigned long channel_3_fs = 1500;  //elev
unsigned long channel_4_fs = 1500;  //rudd
unsigned long channel_5_fs = 1000;  //gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 1000;  //aux1

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
float i_limit = 3.0;    //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 20.0;   //Max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
float maxPitch = 20.0;  //Max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
float maxYaw = 160.0;   //Max yaw rate in deg/sec

float Kp_roll_angle = 0.1;    //Roll P-gain - angle mode
float Ki_roll_angle = 0.1;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (if using controlANGLE2(), set to 0.0)
float B_loop_roll = 0.9;      //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.0;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.00;  //Pitch D-gain - angle mode (if using controlANGLE2(), set to 0.0)
float B_loop_pitch = 0.9;     //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

float Kp_roll_rate = 0.15;     //Roll P-gain - rate mode
float Ki_roll_rate = 0.2;      //Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;   //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
float Kp_pitch_rate = 0.15;    //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;     //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002;  //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)





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
unsigned int channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned int channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;


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
float q0 = 1.0f;  //initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Normalized desired state:
float spd_des = 0, roll_des, pitch_des, yaw_des = 0, pos_des = 0;
int motorActive = 0;
float pos_curr = 0;
float roll_passthru, pitch_passthru, yaw_passthru;

//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;


//Mixer

int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;



//========================================================================================================================//
//                                                      VOID SETUP                                                        //
//========================================================================================================================//


int currState = 0;
int stateStepID = 0;
void gotoState(int state) {
  currState = state;
  stateStepID = 0;
  // motorDriver.resetPosition();
  DEBUG_TELEMETRY.print("gotoState:");
  DEBUG_TELEMETRY.println(state);
}
void loopState() {

  if (currState == 4)  //Tag searching sequence
  {
    // // DEBUG_TELEMETRY.print("currState==4:");
    // if (stateStepID == 0)  // find tag drection stage
    // {
    //   // DEBUG_TELEMETRY.println(camh7data.connectCount);
    //   // if (camh7data.connectCount != 0) {
    //   //   float distance = sqrt(camh7data.y * camh7data.y + camh7data.x * camh7data.x);
    //   //   DEBUG_TELEMETRY.print("distance:");
    //   //   DEBUG_TELEMETRY.println(distance);
    //   //   // if(distance<15)stateStepID=3;
    //   //   float tagBearing = 0;
    //   //   if (camh7data.y > 0)
    //   //     tagBearing = atan(camh7data.x / camh7data.y) * 57.2958;
    //   //   else if (camh7data.y < 0)
    //   //     tagBearing = atan(camh7data.x / camh7data.y) * 57.2958 + 180;
    //   //   else if (camh7data.x < 0) tagBearing = -90;
    //   //   else tagBearing = 90;
    //   //   yaw_des = yaw_IMU + tagBearing;
    //   //   if (yaw_des > 180.0) yaw_des -= 360.0;
    //   //   DEBUG_TELEMETRY.print(yaw_des);
    //   //   // motorDriver.SetControlValue(yaw_des,0);
    //   //   stateStepID = 1;
    //   //   DEBUG_TELEMETRY.print("Step up:");
    //   //   DEBUG_TELEMETRY.println(stateStepID);

    //   // } else {
    //   //   DEBUG_TELEMETRY.println("Tag not detected ");
    //   // }
    // }
    // if (stateStepID == 1)  // rotate to tag drection stage
    // {
    //   float distance = sqrt(camh7data.y * camh7data.y + camh7data.x * camh7data.x);
    //   DEBUG_TELEMETRY.print("distance:");
    //   DEBUG_TELEMETRY.println(distance);
    //   DEBUG_TELEMETRY.print("Step 1:");
    //   DEBUG_TELEMETRY.println(yaw_des - yaw_IMU);
    //   if (abs(yaw_des - yaw_IMU) < 0.5)
    //     if (motorDriver.getRotSpeed() < 0.05)
    //       if (camh7data.connectCount != 0) {
    //         stateStepID = 2;
    //         DEBUG_TELEMETRY.print("Step up:");
    //         DEBUG_TELEMETRY.println(stateStepID);
    //       }
    // }
    // if (stateStepID == 2)  // move to tag position stage
    // {
    //   if (abs(yaw_des - yaw_IMU) < 0.5)
    //     if (motorDriver.getRotSpeed() < 0.05)
    //       if (camh7data.connectCount != 0) {

    //         // motorDriver.SetControlValue(yaw_IMU,camh7data.y);
    //         // stateStepID=2;
    //         DEBUG_TELEMETRY.print(camh7data.y);
    //         DEBUG_TELEMETRY.print("Step up");
    //         DEBUG_TELEMETRY.println(stateStepID);

    //       } else {
    //         DEBUG_TELEMETRY.println("Tag not detected ");
    //       }
    // }
  }
}
RobotDriver *robot;
void setup() {
  
  Serial.begin(115200);          //usb serial
  // DEBUG_TELEMETRY.begin(57600);  //telemetry serial
  Serial.print("Setup started");
  delay(200);
  
  //Initialize radio communication
  // radioSetup();

  DPRINTF("IMU timer started");
  robot= new RobotDriver();
  //Set radio channels to default (safe) values before entering main loop
  // channel_1_pwm = channel_1_fs;
  // channel_2_pwm = channel_2_fs;
  // channel_3_pwm = channel_3_fs;
  // channel_4_pwm = channel_4_fs;
  // channel_5_pwm = channel_5_fs;
  // channel_6_pwm = channel_6_fs;
  // indicateErrorLed(0);
  prev_time = 0;
  timer_data_input.begin(inputDataUpdate, 300);  //
  timer_control_loop.begin(controlUpdate, int(DT_CONTROL * 1000000));
}



int debugID = 0;

// uint8_t udpBuff[300];
int commandBuffIndex = 0;
//========================================================================================================================//
//                                                       MAIN LOOP                                                        //
//========================================================================================================================//

bool manualMode = false;
int loopRatePeriodUS = 50000;
int loopCountActive = 0;
void loop() {
  current_time = micros();            //looprate limiter
  dt = (current_time - prev_time);    //
  if (dt < loopRatePeriodUS) return;  //
  if (dt > loopRatePeriodUS + 10) {
    DPRINTF("loop too slow error:");
    DPRINTLN(dt);
  }

  prev_time = current_time;  //
                             //  if(imu.noMotionCount>100)//200ms
                             //  digitalWrite(13,HIGH);
                             //  else  digitalWrite(13,LOW);

  loopBlink(); //indicate we are in main loop with short blink every 1.5 seconds

  if (1) {
    // getCommandsRadio();


    if (channel_5_pwm < 1500) {
      // imu.resetYaw();
      // motorDriver.robotPosition=0;
    }
    // motorDriver.isActive=motorActive;
    // if (motorDriver.isActive == false) {

    //   motorDriver.resetPosition();
    //   imu.resetYaw();
    //   loopCountActive = 0;
    //   // Serial.println("inactive");
    // } else {
    //   loopCountActive++;
    //   loopState();
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
    // }
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
  Wire.setClock(600000);  //Note this is 2.5 times the spec sheet 400 kHz max...

  mpu6050.initialize();

  while (mpu6050.testConnection() == false) {
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
  mpu9250.setSrd(0);  //sets gyro and accel read to 1khz, magnetometer read to 100hz
#endif
}
void updateSensors() {

  // // IMUData measure = imu.getMeasurement();
  // AccX = measure.accY;
  // AccY = measure.accY;
  // AccZ = measure.accZ;
  // GyroX = measure.gyroX * 57.2958;
  // GyroY = measure.gyroY * 57.2958;
  // GyroZ = measure.gyroZ * 57.2958;
  // // yaw_IMU = measure.gyroyaw*57.2958;
  // yaw_IMU = measure.gyroyaw;
  // Serial.print(yaw_IMU);
  // Serial.print(" ");
  // Serial.print(measure.yaw-170.3);
  // Serial.print(" ");
  // Serial.println(imu.yawCalcMode-100);
}



void getCommandsRadio() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
     Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of
     the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which
     is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the alues are pulled from the SBUS library directly.
     The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise.
  */

  //Low-pass the critical commands and update previous values
  float b = 0.2;  //lower=slower, higher=noiser
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
  float diffParam = (param_max - param_min) / (fadeTime * loopFreq);  //difference to add or subtract from param for each loop iteration for desired fadeTime

  if (state == 1) {  //maximum param bound desired, increase param by diffParam for each loop iteration
    param = param + diffParam;
  } else if (state == 0) {  //minimum param bound desired, decrease param by diffParam for each loop iteration
    param = param - diffParam;
  }

  param = constrain(param, param_min, param_max);  //constrain param within max bounds

  return param;
}

// float switchRollYaw(int reverseRoll, int reverseYaw) {

//   float switch_holder;

//   switch_holder = yaw_des;
//   yaw_des = reverseYaw * roll_des;
//   roll_des = reverseRoll * switch_holder;
// }


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
  } else {
    Serial.println("Calibration Unsuccessful. Please reset the board and try again.");
  }

  while (1)
    ;  //halt code so it won't enter main loop until this function commented out
#endif
  Serial.println("Error: MPU9250 not selected. Cannot calibrate non-existent magnetometer.");
  while (1)
    ;  //halt code so it won't enter main loop until this function commented out
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
    digitalWrite(13, blinkAlternate); //pin 13 is built in LED

    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
    } else if (blinkAlternate == 0) {
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
  // Serial.println(imu.gyroZBiasCompensation * 1000);
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
static void inputDataUpdate() {

  // updateCommandBus();  //read Serial Commands
  robot->update();
}
static void controlUpdate() {
  // motorDriver.update(yaw_IMU);
  // robot->calculateControlLoop();
}
