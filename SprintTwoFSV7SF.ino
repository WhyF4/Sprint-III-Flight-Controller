#include "ESC.h"               //ESC
#include <ESP32Servo.h>        //ESC  
#include <Wire.h> 
#include <Adafruit_Sensor.h>   //IMU
#include <Adafruit_BNO055.h>   //IMU
#include <utility/imumaths.h>  //IMU   
#include <Math.h>

#define red 18 
#define grn 19
//#define blu 
#define escFLpin 32
#define escFRpin 33
#define escBLpin 5
#define escBRpin 25
#define yawPin 13
#define throttlePin 12
#define pitchPin 14
#define rollPin 27

#define BNO055_SAMPLERATE_DELAY_MS (1)
Adafruit_BNO055 bno = Adafruit_BNO055();

ESC escFL (escFLpin, 1000, 2000, 500); // ESC_Name (PIN, Minimum Value, Maximum Value, Arm Value)
ESC escFR (escFRpin, 1000, 2000, 500); 
ESC escBL (escBLpin, 1000, 2000, 500); 
ESC escBR (escBRpin, 1000, 2000, 500); 

//general stuff
double dt;
unsigned long current_time, prev_time, t2, t1;
int FLmotor, FRmotor, BLmotor, BRmotor;
int pitch, roll, yaw, throttle;
float fl, fr, bl, br;
int flScaled, frScaled, blScaled, brScaled;
bool calib, armed, disc, prevDisc;
unsigned long previousMillis2;
int blink, val, output;

//imu
float iRoll, iPitch, iYaw;
float iRollPrev, iPitchPrev, iYawPrev;
float lpIMU = 0.1;

//transmitter
int tRoll, tPitch, tThrottle, tYaw; // t = transmitter
volatile unsigned long rt1, rt2, pt1, pt2, tt1, tt2, yt1, yt2, ft1, ft2;
bool rollFlag, pitchFlag, throttleFlag, yawFlag;
float throDes, rollDes, pitchDes, yawDes;
float tThrottlePrev, tRollPrev, tPitchPrev, tYawPrev;
float maxRoll = 30.0; // deg
float maxPitch = 30.0; // deg
float maxYaw = 160.0; // deg/s
float lpTrans = 0.1;
int oneCnt;
int previousMillis, prevOneCnt;
bool updated, dontCheck;
unsigned long startTime, elapsedTime;

//PID
float iLimit = 25.0;
float errorRoll, integralRoll, integralRollPrev, derivativeRoll, errorRollPrev, rollPID;
float KpRollAngle = 0.0; //.2
float KiRollAngle = 0.0; //.3
float KdRollAngle = 0.0; //.05

float errorPitch, integralPitch, integralPitchPrev, derivativePitch, errorPitchPrev, pitchPID;
float KpPitchAngle = 0.12; //.2
float KiPitchAngle = 0.0; //.3
float KdPitchAngle = 0.1; //.05

float yawPID = 0;



void setup() {
  Serial.begin(115200);
  
  pinMode(escFLpin, OUTPUT); //esc
  pinMode(escFRpin, OUTPUT); //esc
  pinMode(escBLpin, OUTPUT); //esc
  pinMode(escBRpin, OUTPUT); //esc
  pinMode(red, OUTPUT);  //led
  pinMode(grn, OUTPUT);  //led
//  pinMode(blu, OUTPUT);  //led              //not currently using
  pinMode(yawPin, INPUT);   //transmitter
  pinMode(throttlePin, INPUT);  //transmitter
  pinMode(pitchPin, INPUT);  //transmitter
  pinMode(rollPin, INPUT);  //transmitter  

  attachInterrupt(digitalPinToInterrupt(throttlePin), throttleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pitchPin), pitchInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rollPin), rollInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(yawPin), yawInterrupt, CHANGE);

  

  bno.begin();
  armESC();
  bno.setExtCrystalUse(true);

  delay(3000);
}

void loop() {  
        
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  getTransmitterData();

  if (!disc) {
    if (!calib) { 
    throttleDown();     
    displayCalStatus();    
    }
    else {              
      getIMUdata();      
      getDesState();
      computePID();
      controlMixer();
      scaleCommands();
      if (armed) {       
        outputCommands();        
      }
      else {        
        throttleDown();        
      }
    }
  }
  else {
    throttleDown();
  }
  
  loopRate(300);

//  Serial.println(dt*1000000.0);

/*
  Serial.print("  Disc: "); //state machine
  Serial.print(disc);
  Serial.print("  Calib: ");
  Serial.print(calib);  
  Serial.print("  armed: ");
  Serial.println(armed);  

  Serial.print(rollDes); //PID
  Serial.print(" ");
  Serial.print(iRoll);
  Serial.print(" ");
  Serial.println(errorRoll);
 
  Serial.print(KpPitchAngle*errorPitch);  //PID
  Serial.print(" ");
  Serial.print(KiPitchAngle*integralPitch);
  Serial.print(" ");
  Serial.print(KdPitchAngle*derivativePitch);
  Serial.print(" ");
  Serial.print(pitchPID);
  Serial.println();

  Serial.print(flScaled);  //motor outputs
  Serial.print(" ");
  Serial.print(frScaled);
  Serial.print(" ");
  Serial.print(blScaled);
  Serial.print(" ");
  Serial.print(brScaled);
  Serial.println();
*/  
  


}



void computePID() {

  errorRoll = rollDes - iRoll;
  integralRoll = integralRollPrev + errorRoll*dt;
  if (tThrottle <= 1100) {
    integralRoll = 0;
  }
  integralRoll = constrain(integralRoll, -iLimit, iLimit); //prevent unsafe buildup
  derivativeRoll = (errorRoll - errorRollPrev) / dt;
  rollPID = .01*(KpRollAngle*errorRoll + KiRollAngle*integralRoll - KdRollAngle*derivativeRoll);

  errorPitch = pitchDes - iPitch;
  integralPitch = integralPitchPrev + errorPitch*dt;
  if (tThrottle <= 1100) {
    integralPitch = 0;
  }
  integralPitch = constrain(integralPitch, -iLimit, iLimit);
  derivativePitch = (errorPitch - errorPitchPrev) / dt;
  pitchPID = .01*(KpPitchAngle*errorPitch + KiPitchAngle*integralPitch - KdPitchAngle*derivativePitch);

  integralRollPrev = integralRoll;
  errorRollPrev = errorRoll;
  integralPitchPrev = integralPitch;
  errorPitchPrev = errorPitch;
}



void getIMUdata() {

  imu::Quaternion q = bno.getQuat();
  




  iRoll = ((1.0 - lpIMU) * iRollPrev) + (lpIMU * iRoll);
  iPitch = ((1.0 - lpIMU) * iPitchPrev) + (lpIMU * iPitch);
  iYaw = ((1.0 - lpIMU) * iYawPrev) + (lpIMU * iYaw);

  iRollPrev = iRoll;
  iPitchPrev = iPitch;
  iYawPrev = iYaw;
  
} 



void getTransmitterData() {
  
  if (millis() - previousMillis >= 250) {
    previousMillis = millis();
    prevOneCnt = oneCnt;
    updated = true;
  }
  else {
    updated = false;
  }
  if (updated) {
    dontCheck = true;
    startTime = millis();
  }
  else if (elapsedTime >= 75) {
    dontCheck = false;
  }  
  if (startTime != 0) {
    elapsedTime = millis() - startTime;
  }
  if (!dontCheck) {
    if (prevOneCnt == oneCnt) {
      disc = true;
      analogWrite(grn, 0);
      blinkLED(175, red, 150);
      armed = false;
    }
    else if (prevOneCnt != oneCnt) {
      disc = false;
    }    
  }
  if (throttleFlag) {
    throttleFlag = false;
    tThrottle = tt2 - tt1;   
  } 
  if (pitchFlag) {
    pitchFlag = false;
    tPitch = pt2 - pt1;
  }  
  if (rollFlag) {
    rollFlag = false;
    tRoll = rt2 - rt1;
  }
  if (yawFlag) {
    yawFlag = false;
    tYaw = yt2 - yt1;
  }

  tThrottle = ((1.0 - lpTrans) * tThrottlePrev) + (lpTrans * tThrottle);
  tRoll = ((1.0 - lpTrans) * tRollPrev) + (lpTrans * tRoll);
  tPitch = ((1.0 - lpTrans) * tPitchPrev) + (lpTrans * tPitch);
  tYaw = ((1.0 - lpTrans) * tYawPrev) + (lpTrans * tYaw);
  
  tThrottlePrev = tThrottle;
  tRollPrev = tRoll;
  tPitchPrev = tPitch;
  tYawPrev = tYaw;  

  if (prevDisc & !disc) {
    startTime = millis();
  }
  if (startTime != 0) {
    elapsedTime = millis() - startTime;
  }
  
  if (!disc) {
    if (tYaw <= 1050 & tThrottle <= 1050 & armed) {  
      armed = false;
    }  
    if (tYaw >= 1950 & tThrottle <= 1050 & !armed) {     
      armed = true;
      analogWrite(red, 0);
      analogWrite(grn, 150);
    }

    if (!armed) {
      analogWrite(red, 0);
      blinkLED(250, grn, 150);  
    }
    
    if (prevDisc and !disc) {
      throttleDown();
      delay(3000);
    }
  }
  prevDisc = disc; 
}



void getDesState() {
  //normalize and constrain transmitter values

  throDes = (tThrottle - 1000.0) / 1000.0; //between 0 and 1
  rollDes = (tRoll - 1500.0) / 500.0; //between -1 and 1
  pitchDes = (tPitch - 1500.0) / 500.0; //bewteen -1 and 1
  yawDes = (tYaw - 1500.0) / 500.0; //between -1 and 1;

  throDes = constrain(throDes, 0.0, 1.0); 
  rollDes = constrain(rollDes, -1.0, 1.0) * maxRoll;
  pitchDes = constrain(pitchDes, -1.0, 1.0) * maxPitch;
  yawDes = constrain(yawDes, -1.0, 1.0) * maxYaw; 
}



void controlMixer() {
  fl = throDes - pitchPID + rollPID + yawPID;
  fr = throDes - pitchPID - rollPID - yawPID;
  bl = throDes + pitchPID + rollPID - yawPID;
  br = throDes + pitchPID - rollPID + yawPID; 
}

void scaleCommands() {
  flScaled = fl*1000 + 1000;
  frScaled = fr*1000 + 1000;
  blScaled = bl*1000 + 1000;
  brScaled = br*1000 + 1000;

  flScaled = constrain(flScaled, 1000, 2000);
  frScaled = constrain(frScaled, 1000, 2000);
  blScaled = constrain(blScaled, 1000, 2000);
  brScaled = constrain(brScaled, 1000, 2000);
}

void outputCommands() {
  throttleLimit(flScaled);
  escFL.speed(output);
  Serial.print(output);
  Serial.print(" ");
  throttleLimit(frScaled);
  escFR.speed(output);
  Serial.print(output);
  Serial.print(" ");
  throttleLimit(blScaled);
  escBL.speed(output);
  Serial.print(output);
  Serial.print(" ");
  throttleLimit(brScaled);
  escBR.speed(output);
  Serial.print(output);
  Serial.println(" ");
 
}


void throttleInterrupt() {
  if (digitalRead(throttlePin) == HIGH) {
    tt1 = micros();
  }
  else {
    tt2 = micros();
    throttleFlag = true;
  }
  if (throttleFlag == 1) {
    oneCnt++;
  }
}
void pitchInterrupt() {
  if (digitalRead(pitchPin) == HIGH) {
    pt1 = micros();
  }
  else {
    pt2 = micros();
    pitchFlag = true;
  }
}
void rollInterrupt() {
  if (digitalRead(rollPin) == HIGH) {
    rt1 = micros();
  }
  else {
    rt2 = micros();
    rollFlag = true;
  }
}
void yawInterrupt() {
  if (digitalRead(yawPin) == HIGH) {
    yt1 = micros();
  }
  else {
    yt2 = micros();
    yawFlag = true;
  }
}

void throttleDown() {
  escFL.speed(1000);
  escFR.speed(1000);
  escBL.speed(1000);
  escBR.speed(1000);  
}

void armESC() {
  escFL.arm();
  escFR.arm();
  escBL.arm();
  escBR.arm();
}


void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void displayCalStatus(void) {
  int minCal;
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
//  Serial.print("\t");
  if (!system)
  {
//      Serial.print("! ");
  }
//  Serial.print("Sys:");
//  Serial.print(system, DEC);
//  Serial.print(" G:");
//  Serial.print(gyro, DEC);
//  Serial.print(" A:");
//  Serial.print(accel, DEC);
//  Serial.print(" M:");
//  Serial.println(mag, DEC);

  if (system == 3 & gyro == 3 & accel == 3 & mag == 3) {
//    Serial.println("System fully calibrated");
    calib = true;
  }
  else {
    calib = false;
  }
  
  if (!calib) {
    if (accel == 0 or 1 or 2) {
      blinkLED(250, grn, 150);
      blinkLED(250, red, 150);
    }
    else {
      analogWrite(grn, 0);
      analogWrite(red, 0);
    }    
  }
  else {
    analogWrite(grn, 0);
    analogWrite(red, 0);  
  }
}

void blinkLED(int interval, int ledpin, int brightness) {
  if(millis()-previousMillis2>=interval){previousMillis2=millis();blink++;}
  if(blink%2==0){val=brightness;}else{val=0;}
  analogWrite(ledpin, val);
}

int throttleLimit(int motorVal) {
  if (!armed) {
    output = 1000;
  }
  else if (motorVal <= 1100 & armed){
    output = 1100;
  }
  else if (motorVal >= 2000 & armed) {
    output = 2000;
  }
  else {
    output = motorVal;
  }
  return output;
}
