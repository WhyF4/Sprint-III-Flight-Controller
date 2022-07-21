#include "ESC.h"               //ESC
#include <ESP32Servo.h>        //ESC 
#include <Wire.h> 
#include <Adafruit_Sensor.h>   //IMU
#include <Adafruit_BNO055.h>   //IMU
#include <utility/imumaths.h>  //IMU
#include <PID_v1.h>            //PID    //one, if not mulitple of these libraries could very well be causing the code to significantly slow down

#define red 18 
#define grn 19
//#define blu 
#define escFLpin 33
#define escFRpin 32
#define escBLpin 35
#define escBRpin 34
#define yawPin 27
#define throttlePin 12
#define pitchPin 14
#define rollPin 13

#define BNO055_SAMPLERATE_DELAY_MS (1)
Adafruit_BNO055 bno = Adafruit_BNO055();

ESC escFL (escFLpin, 1000, 2000, 500); // ESC_Name (PIN, Minimum Value, Maximum Value, Arm Value)
ESC escFR (escFRpin, 1000, 2000, 500); 
ESC escBL (escBLpin, 1000, 2000, 500); 
ESC escBR (escBRpin, 1000, 2000, 500); 

int t1, t2;
int tRoll, tPitch, tThrottle, tYaw; // t = transmitter
int rollFL, rollFR, rollBL, rollBR;
int pitchFL, pitchFR, pitchBL, pitchBR;
int yawFL, yawFR, yawBL, yawBR;
int FLmotor, FRmotor, BLmotor, BRmotor;
int throttleTwo, yawTwo, pitchTwo, rollTwo;
int pitch, roll, yaw, throttle;
int output, blink, val, value, cnt = 3;
bool disc = false;
bool armed = false;
long previousMillis = 0, previousMillis2 = 0, previousMillis3 = 0, previousMillis4 = 0;
double iPitch, iRoll, iYaw, oldiPitch, oldiRoll, oldiYaw; // i = IMU 

double pitchPIDsetpoint, pitchPIDinput, pitchPIDoutput;     //PID values
const float pKp = 0;
const float pKi = 0;
const float pKd = 0;

double rollPIDsetpoint, rollPIDinput, rollPIDoutput;
const float rKp = 0;
const float rKi = 0;
const float rKd = 0;

PID pitchPID(&pitchPIDinput, &pitchPIDoutput, &pitchPIDsetpoint, pKp, pKi, pKd, DIRECT);    //init PID
PID rollPID(&rollPIDinput, &rollPIDoutput, &rollPIDsetpoint, rKp, rKi, rKd, DIRECT);

void setup() 
  Serial.begin(115200);
  Serial.println("Starting Up");

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
  
  if(!bno.begin()) { Serial.print("Problem wit da imu"); }
  
  pitchPIDinput = 0;
  pitchPIDsetpoint = 0;
  pitchPID.SetOutputLimits(-300,300); 
  pitchPID.SetMode(AUTOMATIC);
  rollPIDinput = 0;
  rollPIDsetpoint = 0;
  rollPID.SetOutputLimits(-300,300); 
  rollPID.SetMode(AUTOMATIC);
    
  escFL.arm();
  escFR.arm();
  escBL.arm();
  escBR.arm();
 
  delay(2500);
  bno.setExtCrystalUse(true);
  Serial.println("Startup Finished");

}

int getEuler() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);   //get data from BNO055
  iYaw = euler.x();
  iPitch = euler.y();
  iRoll = euler.z();
  return iYaw, iPitch, iRoll;
}

int bounds(int value, int lower, int upper) {    //This may not be needed in the future
  if (value >= upper) { value = upper; }
  else if (value <= lower) { value = lower; }
  else { value = value; }
  return value;
}

int deadzone(int value1, int center, int dz) { //DZ is center += dz                 //This may not be needed in the future
  if (value1 <= (center - dz) or value1 >= (center + dz)) { value1 = value1; }
  else { value1 = center; }
  return value1;
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

int throttleDown() {
  escFL.speed(1000);
  escFR.speed(1000);
  escBL.speed(1000);
  escBR.speed(1000);  
}

void loop() {

  Serial.println(t2 - t1);  //calculate loop execution time

  t1 = millis();

  if(millis()-previousMillis2>=250){previousMillis2=millis();blink++;}    //code for blinking LED without delay()
  if(blink%2==0){val=255;}else{val=0;}
  
  if (!disc) {                                //only listen to throttle channel if disconnected
    tRoll = pulseIn(rollPin, HIGH);           //throttle channel is what shows whether or not receiever is connected or not
    tPitch = pulseIn(pitchPin, HIGH);
    tYaw = pulseIn(yawPin, HIGH);   
  }
  tThrottle = pulseIn(throttlePin, HIGH);
  
/*
  tRoll = deadzone(bounds(rawRoll, 930, 1875), 1405, 10);     //This may not be needed in the future
  tPitch = deadzone(bounds(rawPitch, 930, 1875), 1405, 10);
  tYaw = deadzone(bounds(rawYaw, 930, 1875), 1405, 10);
  tThrottle = deadzone(bounds(rawThrottle, 930, 1875), 930, 100);
*/

  if (tYaw <= 1020 & tThrottle <= 1020 & tRoll <= 1020 & tPitch >= 1970 & !armed) {       //Arming procedure on Transmitter
    if (millis() - previousMillis3 >= 1000) {
      previousMillis3 = millis();
      armed = true;
      analogWrite(grn, 255);
      analogWrite(red, 0);
      }
    }
  if (tYaw >= 1970 & tThrottle <= 1020 & tRoll >= 1970 & tPitch >= 1970 & armed) {        //Disarming procedure on Transmitter
    if (millis() - previousMillis3 >= 1000) {
      previousMillis3 = millis();
      throttleDown();
      armed = false;
      analogWrite(grn, 0);
      analogWrite(red, 255);
      }
    }   
    
  if (tThrottle == 0) {       //check if radio is connected
    throttleDown();
    cnt = 0;
    Serial.println("RADIO DISCONNECTED, MOTORS THROTTLED DOWN");  
    disc = true;
  } 
  else {
    Serial.println("Radio Connected");    //if radio connected, do normal IMU and PID calculations
    analogWrite(red, 255);
    analogWrite(grn, 255);

    getEuler(); 
    iPitch = (iPitch *.15) + (oldiPitch *.85);    //filter to eliminate noise in IMU data
    iRoll = (iRoll *.15) + (oldiRoll *.85);
    iYaw = (iYaw *.15) + (oldiYaw *.85);
    oldiPitch = iPitch;
    oldiRoll = iRoll;  
    oldiYaw = iYaw;   

    pitchPIDinput = iPitch;
    pitchPIDsetpoint = map(tPitch, 930, 1875, -20, 20);       //PID calculations; This could possibly be slowing down code
    pitchPID.Compute();
    rollPIDinput = iRoll;
    rollPIDsetpoint = map(tRoll, 930, 1875, -20, 20);
    rollPID.Compute();
    
    roll = map(tRoll, 1000, 2000, 225, -225) + (rollPIDoutput * -1);        //map values from transmitter and add PID calculations
    pitch = map(tPitch, 1000, 2000, 225, -225) + (pitchPIDoutput * -1);
    yaw = map(tYaw, 1000, 2000, -127, 127);
//    throttle = map(tThrottle, 1000, 2000, 1000, 2000);
    throttle = tThrottle;
    
/*    
    roll = (roll * .3) + (previousRoll * .7);       //This may not be needed in the future
    pitch = (pitch * .3) + (previousPitch * .7);
    yaw = (yaw * .3) + (previousYaw * .7);
    throttle = (throttle * .3) + (previousThrottle * .7);    
    previousRoll = roll;
    previousPitch = pitch;
    previousYaw = yaw;
    previousThrottle = throttle;
*/

    roll = roll * -1; //invert axis
//    pitch = pitch * -1; //invert axis
    
    pitchFL = pitch*-1; pitchFR = pitch*-1; pitchBL = pitch; pitchBR = pitch;     //mixer for each individual axis to covert transmitter and PID controls into motor outputs
    rollFL = roll*-1; rollFR = roll; rollBL = roll*-1; rollBR = roll;
    yawFL = yaw*-1; yawFR = yaw; yawBL = yaw; yawBR = yaw*-1;

    FLmotor = rollFL + pitchFL + yawFL + throttle;    //add together each motor output from each axis + throttle
    FRmotor = rollFR + pitchFR + yawFR + throttle;
    BLmotor = rollBL + pitchBL + yawBL + throttle;
    BRmotor = rollBR + pitchBR + yawBR + throttle;
  }
  if (disc) {               //logic for blinking LED
    if (armed) {
      analogWrite(red, val);
      analogWrite(grn, abs(255 - val));
    }
    else {
      analogWrite(red, val);
      analogWrite(grn, 0);
    }
    if (millis() - previousMillis >= 2500) {        //give a wait time before talking to motors after radio connects
      previousMillis = millis();                    //without this wait time, a single motor can sometimes throttle up (I have no idea why)
      cnt++;                                            
      if (cnt > 2) {
        disc = false;      
      }     
    }    
  }      
  else if (cnt > 2 & armed) {           
    throttleLimit(FLmotor); escFL.speed(output);      //if armed and the wait time is done, then write to motors.
//    Serial.print(output);
//    Serial.print("   ");
    throttleLimit(FRmotor); escFR.speed(output);
//    Serial.print(output);
//    Serial.print("   ");
    throttleLimit(BLmotor); escBL.speed(output);
//    Serial.print(output);
//    Serial.print("   ");
    throttleLimit(BRmotor); escBR.speed(output);
//    Serial.print(output);
//    Serial.println("   ");
    Serial.println("Armed");
    analogWrite(red, 0);
    analogWrite(grn, 255);               
  }   
/*
  Serial.print(yaw);
  Serial.print("   ");
  Serial.print(pitch);
  Serial.print("   ");
  Serial.print(roll);
  Serial.print("   ");
  Serial.print(throttle);
  Serial.println("  ");
  Serial.println(disc);

  Serial.print(iPitch);
  Serial.print("   ");
  Serial.print(pitch);
  Serial.print("     ");
  Serial.print(iRoll);
  Serial.print("   ");
  Serial.print(roll);
 Serial.println("  ");
 
  Serial.print(tPitch);
  Serial.print("   ");
  Serial.print(tRoll);
  Serial.print("     ");
  Serial.print(tYaw);
  Serial.print("   ");
  Serial.print(tThrottle);
  Serial.println("  ");
*/
t2 = millis();

}
