/*

 ************************************************************************************
 * MIT License
 *
 * Copyright (c) 2023 Crunchlabs LLC (IRTurret Control Code)
 * Copyright (c) 2020-2022 Armin Joachimsmeyer (IRremote Library)

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ************************************************************************************
 */

//////////////////////////////////////////////////
//  LIBRARIES  //
//////////////////////////////////////////////////
#include <Arduino.h>
#include <Servo.h>
#include "PinDefinitionsAndMore.h"  // Define macros for input and output pin etc.
#include <IRremote.hpp>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define DECODE_NEC  //defines the type of IR transmission to decode based on the remote. See IRremote library for examples on how to decode other types of remote

//////////////////////////////////////////////////
//  PINS AND PARAMETERS  //
//////////////////////////////////////////////////
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int lastYawDegree = 90;

//this is where we store global variables!
Servo yawServo;    //names the servo responsible for YAW rotation, 360 spin around the base
Servo pitchServo;  //names the servo responsible for PITCH rotation, up and down tilt
Servo rollServo;   //names the servo responsible for ROLL rotation, spins the barrel to fire darts

int yawServoVal;  //initialize variables to store the current value of each servo
int pitchServoVal = 100;
int rollServoVal;

int pitchMoveSpeed = 8;  //this variable is the angle added to the pitch servo to control how quickly the PITCH servo moves - try values between 3 and 10
int yawMoveSpeed = 90;   //this variable is the speed controller for the continuous movement of the YAW servo motor. It is added or subtracted from the yawStopSpeed, so 0 would mean full speed rotation in one direction, and 180 means full rotation in the other. Try values between 10 and 90;
int yawStopSpeed = 90;   //value to stop the yaw motor - keep this at 90
int rollMoveSpeed = 90;  //this variable is the speed controller for the continuous movement of the ROLL servo motor. It is added or subtracted from the rollStopSpeed, so 0 would mean full speed rotation in one direction, and 180 means full rotation in the other. Keep this at 90 for best performance / highest torque from the roll motor when firing.
int rollStopSpeed = 90;  //value to stop the roll motor - keep this at 90

int yawPrecision = 150;   // this variable represents the time in milliseconds that the YAW motor will remain at it's set movement speed. Try values between 50 and 500 to start (500 milliseconds = 1/2 second)
int rollPrecision = 158;  // this variable represents the time in milliseconds that the ROLL motor with remain at it's set movement speed. If this ROLL motor is spinning more or less than 1/6th of a rotation when firing a single dart (one call of the fire(); command) you can try adjusting this value down or up slightly, but it should remain around the stock value (160ish) for best results.

int pitchMax = 175;  // this sets the maximum angle of the pitch servo to prevent it from crashing, it should remain below 180, and be greater than the pitchMin
int pitchMin = 10;   // this sets the minimum angle of the pitch servo to prevent it from crashing, it should remain above 0, and be less than the pitchMax


//////////////////////////////////////////////////
//  S E T U P  //
//////////////////////////////////////////////////
void setup() {
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(9600);  // initializes the Serial communication between the computer and the microcontroller

  pinMode(2, INPUT_PULLUP);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  yawServo.attach(10);    //attach YAW servo to pin 10
  pitchServo.attach(11);  //attach PITCH servo to pin 11
  rollServo.attach(12);   //attach ROLL servo to pin 12

  homeServos();  //set servo motors to home position
}

////////////////////////////////////////////////
//  L O O P  //
////////////////////////////////////////////////

void loop() {
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    int currentYaw = ypr[0] * 180 / M_PI;
    int currentPitch = ypr[1] * 180 / M_PI;
    int currentRoll = ypr[2] * 180 / M_PI;
    yaw(currentYaw);
    pitch(currentPitch);

    byte buttonState = digitalRead(2);
    if (buttonState == LOW) {
      fire();
    }
  }
}

void pitch(int degree) {
  float potentialPitchServoVal = 100 + degree; //Crunchlabs horiz value == 100
  potentialPitchServoVal = potentialPitchServoVal > pitchMax ? pitchMax : potentialPitchServoVal < pitchMin ? pitchMin: potentialPitchServoVal; // bound between min and max
  pitchServo.write(potentialPitchServoVal);
}

void yaw(int degree) {
  float speedMod = 2;
  int delta = lastYawDegree - (degree * speedMod) + 90; //90 = No Movement
  delta = delta > 180 ? 180 : delta < 0 ? 0 : delta; //If delta > 180 then return 180 else if delta < 0 then return 0, else return delta
  if (delta != 90) {
    yawServo.write(delta);
  }
  lastYawDegree = degree;
}

/**
* fire does xyz
*/
void fire() {                                      //function for firing a single dart
  rollServo.write(rollStopSpeed + rollMoveSpeed);  //start rotating the servo
  delay(rollPrecision);                            //time for approximately 60 degrees of rotation
  rollServo.write(rollStopSpeed);                  //stop rotating the servo
  delay(5);                                        //delay for smoothness
  Serial.println("FIRING");
}

void homeServos() {
  yawServo.write(yawStopSpeed);  //setup YAW servo to be STOPPED (90)
  delay(20);
  rollServo.write(rollStopSpeed);  //setup ROLL servo to be STOPPED (90)
  delay(100);
  pitchServo.write(100);  //set PITCH servo to 100 degree position
  delay(100);
  pitchServoVal = 100;  // store the pitch servo value
  Serial.println("HOMING");
}
