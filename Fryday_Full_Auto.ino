/*
  Complete RC Control with..
  Autonomous guidance with simple if statements and variable speeds
  3/14/17 by Brian Patton
  Feel free to do whatever you want with this code example
*/
#include <Servo.h>

// Create Variables to hold the Receiver signals
int Ch1, Ch2, Ch3, Ch4, Ch5, Ch6;
int Rwheel;               // Variable to hold R wheel speed 
int Lwheel;               // Variable to hold L wheel speed
const int lPhoto = A1;    // Left Photoresistor
const int rPhoto = A0;    // Right Photoresistor
const int FLsharpPin = A5;  // Front Left Sharp Sensor
const int FRsharpPin = A4;  // Sharp Sensor
const int LED = 13;       // Onboard LED location
int lPhotoVal;            // Variable to store L photoresistor value
int rPhotoVal;            // Variable to store R photoresistor value
int FLsharpVal;             // Variable to store front left Sharp Sensor value
int FRsharpVal;            // var to store front right sharp value
int valDif;               // Variable to store difference between photo values
int rForward, lForward, rBack, lBack; // Constants for forward/backward PWM for motor
// For open loop drive
int AddRight;
int AddLeft;
// align sensitivity
int alignDiff;
// Chute Variables
const int sharpLeftPin = A6;
const int sharpRightPin = A7;
int sharpLeft;                  // int to store left sharp val
int sharpRight;                 // ... right sharp val
int proxDiff;                   // store difference between wall distances
int lightSensitivity;
int chuteProxSensitivity;       // Set chute sharp disparity
// arm boolean
bool armDeployed;
bool overWall;
// ramp --> chute logic
bool inChute;
// startup and end bool
bool init;

// Create Servo Objects as defined in the Servo.h files
Servo L_Servo;  // Servo DC Motor Driver (Designed for RC cars)
Servo R_Servo;  // Servo DC Motor Driver (Designed for RC cars)
Servo Arm_Servo; // Add servo for arm motor (robotic arm)

void setup() {
  // Set the pins that the transmitter will be connected to all to input
  pinMode(12, INPUT); //I connected this to Chan1 of the Receiver
  pinMode(11, INPUT); //I connected this to Chan2 of the Receiver
  pinMode(10, INPUT); //I connected this to Chan3 of the Receiver
  pinMode(9, INPUT); //I connected this to Chan4 of the Receiver
  pinMode(8, INPUT); //I connected this to Chan5 of the Receiver
  pinMode(7, INPUT); //I connected this to Chan6 of the Receiver
  pinMode(LED, OUTPUT);//Onboard LED to output for diagnostics
  // Attach Speed controller that acts like a servo to the board
  R_Servo.attach(2); //Pin 2...? or Pin 0
  L_Servo.attach(1); //Pin 1
  // init arm
  Arm_Servo.attach(29); //check pinout?
  armDeployed = false;
  overWall = false;
  inChute = false;
  init = false;

  // ***IMPORTANT*** --> VALUES TO TUNE
  rForward = 1580;
  lForward = 1580;
  rBack = 1340;
  lBack = 1340;
  // chute sensitivity
  chuteProxSensitivity = 100;
  lightSensitivity = 125;
  // align sensitivity
  alignDiff = 30;

  //Flash the LED on and Off 10x before entering main loop
  for (int i = 0; i < 10; i++) {
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
  }
  //Flash the LED on and Off 10x End
  Serial.begin(9600);
}


void loop()
{
  ChCheck();
  // autonomousLight();
  // // testing
  checkSensors();
  printSensors();
}

void ChCheck() {
  Ch5 = pulseIn(8, HIGH, 21000); // Capture pulse width on Channel 5
  if (Ch5 > 1600) {
    digitalWrite(LED, HIGH);
    if (!init) {
      initialize(5000); // spot turn for x seconds to turn 180 degrees
      init = true;
    }
    if (!inChute) {
      rForward = 1560;
      lForward = 1560;
      lightSensitivity = 100;
    }
    else {
      rForward = 1580;
      lForward = 1580;
      lightSensitivity = 125;
    }
    autonomousLight();
  }
  else {
    // armDeployed = false;
    // overWall = false;
    // inChute = false;
    // init = false;
    // Serial.println("Open-Loop driving...");
    Ch1 = pulseIn(12, HIGH, 21000); // Capture pulse width on Channel 1
    Ch2 = pulseIn(11, HIGH, 21000); // Capture pulse width on Channel 2
    Ch4 = pulseIn(9, HIGH, 21000); // Capture pulse width on Channel 4
    digitalWrite(LED, LOW);
    DriveServosRC();
  }
}

void DriveServosRC()
{
  if (Ch2 <= 1450) {
    int Ch1_mod = map(Ch1, 1000, 2000, 2000, 1000); // Invert the Ch1 axis to keep the math similar
    AddLeft = 0;
    AddRight = 0;
    if (Ch1_mod > 1550) {
      AddLeft = abs(Ch1_mod-1500);
      AddRight = -abs(Ch1_mod-1500);
    }
    if (Ch1_mod < 1450) {
      AddRight = abs(Ch1_mod-1500);
      AddLeft = -abs(Ch1_mod-1500);
    }
    Lwheel = AddLeft + Ch2;
    Rwheel = AddRight + Ch2;
    SetLimits();
    // PrintRC();
  }
  if (Ch2 > 1450) {
    AddLeft = 0;
    AddRight = 0;
    if (Ch1 > 1550) {
      AddLeft = abs(Ch1-1500);
      AddRight = -abs(Ch1-1500);
    }
    if (Ch1 < 1450) {
      AddRight = abs(Ch1-1500);
      AddLeft = -abs(Ch1-1500);
    }
    Lwheel = AddLeft + Ch2;
    Rwheel = AddRight + Ch2;
    SetLimits();
    // PrintRC();
  }
}

void SetLimits() {
  // remove deadband noise
  if ((Rwheel < 1550) && (Rwheel > 1450)){
    Rwheel = 1500;
  }
  if ((Lwheel < 1550) && (Lwheel > 1450)){
    Lwheel = 1500;
  }

  // Currently w/ ESCs, reverse is half forward frequency
  if (Rwheel > 1500) {
    Rwheel = 1500 + (Rwheel - 1500)/2; 
  }
  if (Lwheel > 1500) {
    Lwheel = 1500 + (Lwheel - 1500)/2; 
  }

  // Set Max Ranges
  if (Lwheel < 1000) {// Can be set to a value you don't wish to exceed
    Lwheel = 1000;    // to adjust maximums for your own robot
  }
  if (Lwheel > 1750) {// Can be set to a value you don't wish to exceed
    Lwheel = 1750;    // to adjust maximums for your own robot
  }
  if (Rwheel < 1000) {// Can be set to a value you don't wish to exceed
    Rwheel = 1000;    // to adjust maximums for your own robot
  }
  if (Rwheel > 1750) {// Can be set to a value you don't wish to exceed
    Rwheel = 1750;    // to adjust maximums for your own robot
  }

  pulseMotors();
}

void pulseMotors() {
  // un-comment the next two line to drive the wheels directly with the MaxLimits Set
  //  R_Servo.writeMicroseconds(Rwheel);
  //  L_Servo.writeMicroseconds(Lwheel);
  //un-comment the next two to map a control range.
  //*** Take the standard range of 1000 to 2000 and frame it to your own minimum and maximum
  //*** for each wheel.
  Rwheel = map(Rwheel, 1000, 2000, 1000, 2000);
  Lwheel = map(Lwheel, 1000, 2000, 1000, 2000);
  // PrintRC();
  R_Servo.writeMicroseconds(Rwheel);
  L_Servo.writeMicroseconds(Lwheel);
  // un-comment this line do display the value being sent to the motors
  //  PrintWheelCalcs(); //REMEMBER: printing values slows reaction times
  Arm_Servo.write(Ch4);

}

// Autonomous stuff

//******************** checkSensors() **************************
// Check value of Sensors         Stop bot if object is close
//**************************************************************
void checkSensors()
{
  // light
  rPhotoVal = analogRead(rPhoto);
  lPhotoVal = analogRead(lPhoto);
  valDif = abs(rPhotoVal - lPhotoVal); // looking for threshold
  // left
  FLsharpVal = analogRead(FLsharpPin);
  for (int i = 0; i <= 3; i++) {
    FLsharpVal = FLsharpVal + analogRead(FLsharpPin);
  }
  FLsharpVal = FLsharpVal / 5;
  // right
  FRsharpVal = analogRead(FRsharpPin);
  for (int i = 0; i <= 3; i++) {
    FRsharpVal = FRsharpVal + analogRead(FRsharpPin);
  }
  FRsharpVal = FRsharpVal / 5;
  // chute
  // Left and Right sharps for chute
  sharpLeft = analogRead(sharpLeftPin);
  for (int i = 0; i <= 3; i++) {
    sharpLeft = sharpLeft + analogRead(sharpLeftPin);
  }
  sharpLeft = sharpLeft / 5;
  // Iteration to average values
  sharpRight = analogRead(sharpRightPin);
  for (int i = 0; i <= 3; i++) {
    sharpRight = sharpRight + analogRead(sharpRightPin);
  }
  sharpRight = sharpRight / 5;
  // Find difference
  proxDiff = abs(sharpLeft - sharpRight); // looking for threshold
  // printSensors();
}

void autonomousLight() {
  checkSensors();
  if ((lPhotoVal > 650) && (rPhotoVal > 650)) {
    // assume no reflection on chute walls
    autonomousChute();
    Serial.println("doing chute!");
  }
  else {
    if (overWall == true) {
      if ((FLsharpVal >= 350) && (FRsharpVal >= 350)) {
        // need careful checks - can prematurely deploy
        Serial.println("stopping!");
        Brake(10);
        if (armDeployed == false) {
          medkit();
          armDeployed = true;
        }
      }
    }
    if (valDif > lightSensitivity) {
      if (lPhotoVal > rPhotoVal) {
        TLeftPivot(10, 1);
        Serial.println("turning left!");
      }
      else {
        TRightPivot(10, 1);
        Serial.println("turning right!");
      }
    }
    else {
      Forward(10);
      Serial.println("going forward...");
    }
  }
}
void autonomousChute() {
  Serial.println("doing chute!");
  checkSensors();
  if (overWall == false) {
    if ((FLsharpVal >= 350) && (FRsharpVal >= 350)) {
      Serial.println("Traversing Wall!");
      Brake(1000);
      // Fxn for climbing wall ~ time to go forward for, and pwm signal to send
      climbWall(2750, 1600);
      Brake(1000);
      climbWall(4000, 1575);
      Brake(1000); // signal finish over wall, wait 5 seconds before continue
      overWall = true;
    }
  }
  if ((sharpLeft >= 200) && (sharpRight >= 200)) {
    inChute = true;
    Serial.println("inside chute!");
  }
  if ((sharpLeft <= 200) || (sharpRight <= 200)) {
    if (inChute) {
      Serial.println("inside chute!");
    }
    else {
      Serial.println("outside chute!");
      Forward(10);
    }
  }
  else {
    if (proxDiff > chuteProxSensitivity) {
      if (sharpLeft > sharpRight) { 
        Serial.println("Turn Right!");
        TRightPivot(10, 1);
      }
      else {
        Serial.println("Turn Left!");
        TLeftPivot(10, 1);
      }
    }
    else {
      Forward(10);
      Serial.println("Proceed Forward!");
    }
  }
}

// Autonomous drive fxns
void climbWall(int duration, int speed) {
  R_Servo.writeMicroseconds(speed);
  L_Servo.writeMicroseconds(speed);
  delay(duration);
}
void medkit() {
  Serial.println("deploy medkit!");
  Arm_Servo.writeMicroseconds(1420);
  delay(5500);
  Arm_Servo.writeMicroseconds(1500);
}
void Forward(int Dlay)
{
  R_Servo.writeMicroseconds(rForward);  // sets the servo position
  L_Servo.writeMicroseconds(lForward);   // sets the servo position
  delay(Dlay);
}
void TLeftPivot(int Dlay, int direction)
{
  if (direction == 1){
    R_Servo.writeMicroseconds(rForward);  // sets the servo position
  }
  if (direction == -1) {
    R_Servo.writeMicroseconds(rBack);  // sets the servo position
  }
  L_Servo.writeMicroseconds(1500);   // sets the servo position
  delay(Dlay);
}
void TRightPivot(int Dlay, int direction)
{
  R_Servo.writeMicroseconds(1500);  // sets the servo position
  if (direction == 1){
    L_Servo.writeMicroseconds(lForward);   // sets the servo position
  }
  if (direction == -1) {
    L_Servo.writeMicroseconds(lBack);   // sets the servo position
  }
  delay(Dlay);
}
void spotLeft(int Dlay)
{
  R_Servo.writeMicroseconds(rForward);  // sets the servo position
  L_Servo.writeMicroseconds(lBack);   // sets the servo position
  delay(Dlay);
}
void spotRight(int Dlay)
{
  R_Servo.writeMicroseconds(rBack);  // sets the servo position
  L_Servo.writeMicroseconds(lForward);   // sets the servo position
  delay(Dlay);
}
// IMPORTANT: to cancel out braking effects (small delay)
void Brake(int Dlay)
{
  R_Servo.writeMicroseconds(1500);  // sets the servo position
  L_Servo.writeMicroseconds(1500);   // sets the servo position
  delay(Dlay);
}
void initialize(int Dlay){
  spotLeft(Dlay);
  Brake(500);
}

// Diagnostics
void printSensors() {
  Serial.println("Right Value = " + (String)rPhotoVal);
  Serial.println("Left Value = " + (String)lPhotoVal);
  Serial.println("Difference Value = " + (String)valDif);
  Serial.println("Left Sharp Value = " + (String)sharpLeft);
  Serial.println("Right Sharp Value = " + (String)sharpRight);
  Serial.println("Front Left Sharp Value = " + (String)FLsharpVal);
  Serial.println("Front Right Sharp Value = " + (String)FRsharpVal);
  Serial.println(" ");
  delay(100);
}
void PrintRC()
{ // print out the values you read in:
  Serial.println(" RC Control Mode ");
  Serial.print("Rwheel = ");
  Serial.println(Rwheel);
  Serial.print("Lwheel = ");
  Serial.println(Lwheel);
}