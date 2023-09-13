#include <NewPing.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#pragma once   //Stops multiple opening of same .h files.
#define IR_SENSOR_RIGHT 4
#define IR_SENSOR_LEFT 8


MPU6050 mpu;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
int mpuValue;
boolean mpuValueNegative = false;
float ypr[3];
volatile bool mpuInterrupt = false;
int heading;
int Stop = 1492;
int remainder = 0;
int division = 0;
int headingCounter = 0;
int LeftMotorSpeed = 100;
int RightMotorSpeed = 100;
int lineDetectedCount = 0;
bool followLine = false;
void dmpDataReady()
{
  mpuInterrupt = true;
} //End dmpDataReady.

#define TRIGGER_PIN  12// Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     13// Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters).

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
//Right motor
int enableRightMotor = 9;
int rightMotorPin1 = 10;
int rightMotorPin2 = 11;

//Left motor
int enableLeftMotor = 5;
int leftMotorPin1 = 6;
int leftMotorPin2 = 7;

String readString;
// Viewing from front of BOE BOT Car...
int value = 0;
int CenterDistance = 0; //Initialize center distance.
int LimitDistance = 10; //itnitialize limit distance.
int pos = 0;    // variable to store the servo position

void setup() {
  Wire.begin();
  TWBR = 24;
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setXAccelOffset(-1297); //1293,
  mpu.setYAccelOffset(2424); //2425,
  mpu.setZAccelOffset(1570); // 1572, (1688 factory default for my test chip)
  mpu.setXGyroOffset(-13);  //-13,
  mpu.setYGyroOffset(-55);  //-55,
  mpu.setZGyroOffset(-27);  //-27,
  mpu.setDMPEnabled(true);
  attachInterrupt(0, dmpDataReady, RISING);
  packetSize = mpu.dmpGetFIFOPacketSize();

  Serial.begin(9600);
  mpuValue = 0; //Initialize yaw to zero in order to bypass first heading check in Loop.
  heading = 0; //Initialize to zero.
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);

}
void loop() {
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
  displayWheelInfo();
  Right();
  Forward(); //Start out going forward.

  GetMPUValue(); //Get mpuValue and check if vering left (negative) or right (positive).
  if (abs(mpuValue) != heading)
  {
    //  Serial.println("CheckHeading...");
    //  GetMPUValue();
    CheckHeading(); //Verify still on correct heading course.
    //  mpuValueNegative = false;

    displayWheelInfo();

  } //End If.
  CenterDistance = GetDistanceCenter(); //Check to see if there is an obstacle (at limit).
  if (CenterDistance > LimitDistance) //Clear? Go Forward.
  {
    Forward();

  } //End If.
  else
  {
    Right();
    Reverse();
    Left();
    Forward(); //Start out going forward.
    if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH ) {
      if (lineDetectedCount == 0) {
        // First time line detected, continue moving forward
        Forward(); //Start out going forward.
        lineDetectedCount++;
      } else if (lineDetectedCount == 1) {
        // Second time line detected, stop for 1 second and then start following line
        followLine = true;
        lineDetectedCount++;
      }
      else if (lineDetectedCount == 2) {
        Left();
        Followline();
        CenterDistance = GetDistanceCenter(); //Check to see if there is an obstacle (at limit).
        if (CenterDistance > LimitDistance) //Clear? Go Forward.
        {
          Followline();
        }
        else
        {
          Reverse();
        }
      }
    }
  }
}

void Followline() {
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  //If none of the sensors detects black line, then go straight
  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
  {
    analogWrite(enableRightMotor, abs(RightMotorSpeed));
    analogWrite(enableLeftMotor, abs(LeftMotorSpeed));
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  }
  //If right sensor detects black line, then turn right
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
  {
    analogWrite(enableRightMotor, abs(RightMotorSpeed));
    analogWrite(enableLeftMotor, abs(LeftMotorSpeed));
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  }
  //If left sensor detects black line, then turn left
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
  {
    analogWrite(enableRightMotor, abs(RightMotorSpeed));
    analogWrite(enableLeftMotor, abs(LeftMotorSpeed));
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  }
}
void Right()
{
  Serial.println();
  Serial.println("RIGHT TURN --->>");
  Serial.println();

  headingCounter = headingCounter + 90;
  remainder = abs(headingCounter % 180);  //must stay positive.
  division = abs(headingCounter / 180);   //must stay positive.

  if (remainder == 0 & division % 2 != 0) remainder = 179;
  if (remainder == 0 & division % 2 == 0) remainder = 0;
  heading = remainder;         //Set the heading to the corrected "degree" (0, 90, or 179) values only.


  GetMPUValue();
  displayWheelInfo();

  while (abs(mpuValue - heading) > 5)
  {

    for (pos = Stop; pos < 900; pos += 15) // goes from 0 degrees to 90 degrees
    { // in steps of 1 degree
      analogWrite(enableRightMotor, abs(RightMotorSpeed));
      analogWrite(enableLeftMotor, abs(LeftMotorSpeed));
      digitalWrite(leftMotorPin1, HIGH);
      digitalWrite(leftMotorPin2, LOW);
      digitalWrite(rightMotorPin1, LOW);
      digitalWrite(rightMotorPin2, HIGH);
    } //End For.

    GetMPUValue();  //Required to exit while statement.
    displayWheelInfo();
  } //End While.


  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);

  delay(200);

  Forward();

} //End Right.
void Left()
{

  Serial.println();
  Serial.println("LEFT TURN <<---");
  Serial.println();

  headingCounter = headingCounter - 90;
  remainder = abs(headingCounter % 180);  //must stay positive.
  division = abs(headingCounter / 180);   //must stay positive.

  if (remainder == 0 & division % 2 != 0) remainder = 179;
  if (remainder == 0 & division % 2 == 0) remainder = 0;
  heading = remainder;         //Set the heading to the corrected "degree" (0, 90, or 179) values only.


  GetMPUValue();
  displayWheelInfo();

  while (mpuValue != heading)
  { //Must account for heading = 0.
    if (mpuValue >= (heading - 10) && mpuValue <= (heading + 10)) goto bailout;

    for (pos = Stop; pos > 1410; pos -= 20) // goes from 0 degrees to 90 degrees
      //    for(pos = Stop; pos < 1575; pos += 10)  // goes from 0 degrees to 90 degrees
    { // in steps of 1 degree
      analogWrite(enableRightMotor, abs(RightMotorSpeed));
      analogWrite(enableLeftMotor, abs(LeftMotorSpeed));
      digitalWrite(leftMotorPin1, LOW);
      digitalWrite(leftMotorPin2, HIGH);
      digitalWrite(rightMotorPin1, HIGH);
      digitalWrite(rightMotorPin2, LOW);
      GetMPUValue();
    } //End For.

    GetMPUValue();
    displayWheelInfo();
  } //End While.


bailout:
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);

  delay(200);

  Forward();

} //End Left.
void GetMPUValue()
{
  while (!mpuInterrupt) {
  } //End While. This is necessary during non-interrupt time periods.


  mpuInterrupt = false;
  fifoCount = mpu.getFIFOCount();

  if (fifoCount == 1024) {
    mpu.resetFIFO();
  } //End If.
  else
  {

    if ((fifoCount % packetSize != 0) || (fifoCount < packetSize) )
    {
      mpu.resetFIFO();
    } //End If.
    else
    {

      while (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
      } //End While.

      while (!mpuInterrupt) {
      } //End While.

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


      mpuValue = (ypr[0] * 180 / M_PI);
      //      Serial.println(mpuValue); //Debug.
      if (mpuValue < 0) mpuValueNegative = true;
      //    mpuValue = abs(ypr[0] * 180/M_PI); //Convert angle to a positive number and remove decimal point.
      mpuValue = abs(mpuValue); //Concert to positive.

    } //End Else.

  } //End Else.

} //End GetMPUValue.

/*****************************************************/
/****            CheckHeading                     ****/
/* Check to see if BOE BOT Car is veering off the    */
/* MPU6050 heading value (ie: 0, 90, 180 degrees).   */
/* Adjust from looking from rear of Bot Car.         */
/*****************************************************/
void CheckHeading()
{


  if (mpuValueNegative)
  {
    AdjustRight();
  } //End If.

  else
  {
    AdjustLeft();
  } //End Else.

  mpuValueNegative = false;

} //End CheckHeading.


/*****************************************************/
/****            AdjustLeft                       ****/
/* If the BOE BOT Car starts to drift to the right,  */
/* adjust the direction back towards the left. This  */
/* adjustment is viewed from behind the BOE BOT Car. */
/*****************************************************/
void AdjustLeft() {

  //Viewed from behind BOE BOT Car...
  Serial.println("AdjustLeft..."); //Debug.

  int modifiedmpuValue = 125; //Was 50, 25.

  LeftMotorSpeed = LeftMotorSpeed - modifiedmpuValue;  //Speed up wheel speed.

  RightMotorSpeed = RightMotorSpeed - modifiedmpuValue; //Slow down wheel speed.


  if (LeftMotorSpeed < 1100)
  {
    LeftMotorSpeed = 1100;
  } //Don't exceed max speed value of 1100.

  if (RightMotorSpeed < 1500)
  {
    RightMotorSpeed = 1500;
  }//Don't reach stop value.

  analogWrite(enableRightMotor, abs(RightMotorSpeed));
  analogWrite(enableLeftMotor, abs(LeftMotorSpeed));
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);

  displayWheelInfo();

} //End AdjustLeft.

/*****************************************************/
/****            AdjustRight                      ****/
/* If the BOE BOT Car starts to drift to the left,   */
/* adjust the direction back towards the right. This */
/* adjustment is viewed from behind the BOE BOT Car. */
/*****************************************************/
void AdjustRight() {

  //Viewed from behind BOE BOT Car...
  Serial.println("AdjustRight..."); //Debug.

  int modifiedmpuValue = 125; //Was 50, 25.

  //  LeftWheelSpeed = LeftWheelSpeed + modifiedmpuValue ; //Slow down.
  //  RightWheelSpeed = RightWheelSpeed + modifiedmpuValue; // Speed up.

  RightMotorSpeed = 2300; //Debug.
  LeftMotorSpeed = 1425; //Debug.1400,1450,1425
  /*
    if(RightWheelSpeed > 2300)
    {
    RightWheelSpeed = 2300;
    } //Don't exceed max speed value of 2300.

    if(LeftWheelSpeed > 1488)
    {
    LeftWheelSpeed = 1488;
    }  //Don't reach stop value.

  */
  analogWrite(enableRightMotor, abs(RightMotorSpeed));
  analogWrite(enableLeftMotor, abs(LeftMotorSpeed));
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);

  displayWheelInfo();

} //End AdjustLeft.
int GetDistanceCenter() {
  delay(500);
  int value = GetDistance();
  delay(100);
  if (value == 0) {
    value = 200;
  }
  return value;
} //End GetDistanceCenter.

/*****************************************************/
/****            FORWARD                          ****/
/*****************************************************/
void Forward()
{

  Serial.println("Forward...");

  analogWrite(enableRightMotor, abs(RightMotorSpeed));
  analogWrite(enableLeftMotor, abs(LeftMotorSpeed));
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);


} //End Forward.


/*****************************************************/
/****                  REVERSE                    ****/
/*****************************************************/
void Reverse()
{

  analogWrite(enableRightMotor, abs(RightMotorSpeed));
  analogWrite(enableLeftMotor, abs(LeftMotorSpeed));
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);


} //End Reverse.


/*****************************************************/
/****                   GETDISTANCE               ****/
/* Get the ping distance and convert it to cms.      */
/*****************************************************/
int GetDistance()
{
  delay(30);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  int pingValue = ((uS / US_ROUNDTRIP_CM) * 0.3937);
  if (pingValue == 0)
  {
    pingValue = 200; //Can not have a value of 0. So set to maximum distance.
    //***     Serial.println(pingValue);
    return pingValue;
  } // End If. If distance is infinite then set to 200.
  else
  {
    //***    Serial.println(pingValue);
    return pingValue;
  } //End Else.

} //End GetDistance.


/*****************************************************/
/****                  displayWheelInfo           ****/
/*****************************************************/
void displayWheelInfo() {


  Serial.print("Right...");
  Serial.print(RightMotorSpeed);
  Serial.print("\t");
  Serial.print("Left...");
  Serial.print(LeftMotorSpeed);
  Serial.print("\t");
  Serial.print("mpuValue...");
  Serial.print(mpuValue);
  Serial.print("\t");
  Serial.print("Heading...");
  Serial.print(heading);
  Serial.println("\t");

} //End display WheelInfo.




