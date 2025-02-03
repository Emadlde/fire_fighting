#include <ps5Controller.h>
#include <ESP32Servo.h>
#include "Wire.h"
#include <MPU6050_light.h>
#define SERVO_FORWARD_STEP_ANGLE 1
#define SERVO_BACKWARD_STEP_ANGLE -1

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;

long timer = 0;

int mid_ir = 25;
int right_ir = 33;
int left_ir = 26;

unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 20;  // Update interval in milliseconds


//Right motor
int enableRightMotor = 4;
int rightMotorPin1 = 18;
int rightMotorPin2 = 19;
//Left motor
int enableLeftMotor = 16;
int leftMotorPin1 = 0;
int leftMotorPin2 = 2;




const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;

const int stickDeadZone = 10;

void notify() {

  if (ps5.R2()) {
    int yAxisValue = (ps5.data.analog.stick.ly);  // Left stick - y axis - forward/backward car movement
    int xAxisValue = (ps5.data.analog.stick.rx);  // Right stick - x axis - left/right rotation

    // Apply dead zone for stick drift
    if (abs(yAxisValue) < stickDeadZone) yAxisValue = 0;
    if (abs(xAxisValue) < stickDeadZone) xAxisValue = 0;

    // Map values for throttle and steering
   int throttle = map(yAxisValue, -127, 127, -255, 255);  // Forward/backward control
  int steering = map(xAxisValue, 127, -127, -255, 255);  // Rotation control

    int rightMotorSpeed, leftMotorSpeed;

    // Combine throttle (forward/backward) and steering (rotation)
    rightMotorSpeed = throttle - steering;
    leftMotorSpeed = throttle + steering;

    // Constrain motor speeds to allowable range (-255 to 255)
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);
    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);

    // Directly control the motor pins based on speed and direction
    rotateMotor(rightMotorSpeed, leftMotorSpeed);
  } else {

    if(ps5.Triangle())
    {
      while(1)
      {
          int right = digitalRead(right_ir);
  int mid = digitalRead(mid_ir);
  int left = digitalRead(left_ir);

  Serial.print("Left: ");
  Serial.print(left);
  Serial.print(" Mid: ");
  Serial.print(mid);
  Serial.print(" Right: ");
  Serial.println(right);

  if (left == 0 && right == 0) {
    forward();
  } else if (left == 0 && right == 1) {
    rightTurn();
  } else if (left == 1 && right == 0) {
    leftTurn();
  } else if (left == 1 && right == 1) {
   leftTurn();
  }
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= updateInterval) {
    baseServo.write(30);
    shoulderServo.write(98);
    elbowServo.write(100);
    lastUpdateTime = currentTime;
  }
      }


    }
    analogWrite(enableRightMotor, 0);
    analogWrite(enableLeftMotor, 0);
    // Moving the servos at a controlled rate
    // Reading analog stick values
    int rx = (ps5.data.analog.stick.rx);  // Base       =>  Right stick - x axis
    int ry = (ps5.data.analog.stick.ry);  // Shoulder   =>  Right stick - y axis
    int ly = (ps5.data.analog.stick.ly);  // Elbow      =>  Left stick - y axis
    int lx = (ps5.data.analog.stick.lx);  // Gripper    =>  Left stick - x axis

    // Mapping stick values to servo angles
    int base = map(rx, 127, -127, 0, 180);      // For Base Servo
    int shoulder = map(ry, -127, 127, 0, 180);  // For Shoulder Servo
    int elbow = map(ly, 127, -127, 0, 180);     // For Elbow Servo
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime >= updateInterval) {
      baseServo.write(base);
      Serial.println(base);
      shoulderServo.write(shoulder);
      elbowServo.write(elbow);

      lastUpdateTime = currentTime;
    }
  }
}

void onConnect() {
  Serial.println("Connected!");
}

void onDisConnect() {
  Serial.println("Disconnected!");
}


void setup() {

  pinMode(right_ir, INPUT);
  pinMode(mid_ir, INPUT);
  pinMode(left_ir, INPUT);

  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);


  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  rotateMotor(0, 0);
  // Allocate timers for ESP32 PWM
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Attach servos
  baseServo.setPeriodHertz(50);  // Standard 50hz for servos
  shoulderServo.setPeriodHertz(50);
  elbowServo.setPeriodHertz(50);

  baseServo.attach(5, 500, 2400);       // Attach base servo to pin 27    #TO DO
  shoulderServo.attach(27, 500, 2400);  // Attach shoulder servo to pin 26
  elbowServo.attach(15, 500, 2400);     // Attach elbow servo to pin 25

  Serial.begin(115200);


  // Perform sensor calibration (if not done automatically in mpu.begin())


  // PS5 Controller setup
  ps5.attach(notify);                    // Attach the notify function for controller input
  ps5.attachOnConnect(onConnect);        // Attach connection event handler
  ps5.attachOnDisconnect(onDisConnect);  // Attach disconnection event handler
  ps5.begin("58:10:31:7D:DD:41");        // Initialize PS5 controller with MAC address

  // Wait until the PS5 controller connects
  while (!ps5.isConnected()) {
    Serial.println("PS5 controller not found");
    delay(300);
  }
  Serial.println("Ready.");
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  // Right motor direction control
  if (rightMotorSpeed < 0)  // Reverse
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  } else if (rightMotorSpeed > 0)  // Forward
  {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  } else  // Stop
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }

  // Left motor direction control
  if (leftMotorSpeed < 0)  // Reverse
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  } else if (leftMotorSpeed > 0)  // Forward
  {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  } else  // Stop
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }

  // Simulate motor speed using delay-based PWM control (simple method without PWM libraries)
  analogWrite(enableRightMotor, abs(rightMotorSpeed));  // Replace with suitable analogWrite method
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    // Replace with suitable analogWrite method
}

void forward() {
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(enableRightMotor, 80);
  analogWrite(enableLeftMotor, 80);
  Serial.println("Moving Forward");
}

void rightTurn() {
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(enableRightMotor, 140);
  analogWrite(enableLeftMotor, 140);
  Serial.println("Turning Right");
}

void leftTurn() {
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  analogWrite(enableRightMotor, 140);
  analogWrite(enableLeftMotor, 140);
  Serial.println("Turning Left");
}

void spin() {
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  analogWrite(enableRightMotor, 110);
  analogWrite(enableLeftMotor, 110);
  Serial.println("Spinning");
}

void loop() {
  // Main loop
}
