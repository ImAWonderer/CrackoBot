#include <NewPing.h>
#include <Servo.h> 

// Utrasonic Sensor
#define TRIGGER_PIN         12
#define ECHO_PIN            11
#define SONIC_READ_DELAY    50

// Servo
#define SERVO_PIN           9
#define HEAD_MOVEMENT_DELAY 5

// LED

#define LED_PIN             10

// Motor Contoller
#define IN1                 7
#define IN2                 8
#define ENA                 6
#define IN3                 5  
#define IN4                 4   
#define ENB                 3

// General Variables
#define MAX_DISTANCE        200 // Maximum distance we want to ping for (in centimeters)
#define MIN_DISTANCE        30 
#define MAX_SPEED           100
#define LEFT_WHEEL_DIFF     30  // Percentage of diff from the motors speed

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
Servo headServo; // Variable for the servo that moves the head

int motorModes[][4] = {{LOW,LOW,LOW,LOW},  // Matrix with the different directions that the robot can move to
                       {LOW,HIGH,LOW,HIGH},
                       {HIGH,LOW,HIGH,LOW},
                       {LOW,HIGH,HIGH,LOW},
                       {HIGH,LOW,LOW,HIGH}};

boolean contMovement = true; // When automatic mode is On, indicates if the bot should continue moving
boolean manualMode = true; // Indicates if manual mode is On/Off
boolean ledOn = false; // Indicates if the front LED is On

int motorSpeed = MAX_SPEED; // Init speed to MAX_SPEED
int servoPositions[3] = {180, 90, 1}; // Possible positions for the servo
int distanceCm = 0; // Keeps the value of the distance from the Ultrasonic sensor
int currentHeadPosition = 1; // Current Head Position
int heading = 1; // Variable to set next head position
int direction = 1; // Last head direction movement
char btData = 0; // Data from Serial Bluetooth connection

void setup() {
  Serial.begin(9600); // Init serial port
  headServo.attach(SERVO_PIN); // Init servo pin
  pinMode(ENA, OUTPUT); // Set Pin for motor controller
  pinMode(IN1, OUTPUT); // Set Pin for motor controller
  pinMode(IN2, OUTPUT); // Set Pin for motor controller
  pinMode(ENB, OUTPUT); // Set Pin for motor controller
  pinMode(IN3, OUTPUT); // Set Pin for motor controller
  pinMode(IN4, OUTPUT); // Set Pin for motor controller
  pinMode(LED_PIN, OUTPUT); // init LED pin
  pinMode(13, OUTPUT); // Fix for Arduino board LED to not be always ON
}

// Read Ultrasonic sensor distance
void readSensor() {
  delay(SONIC_READ_DELAY);
  unsigned int sonarRead = sonar.ping();
  distanceCm = sonarRead / US_ROUNDTRIP_CM;
}

// Move head from position "currentHeadPosition" to "heading"
void moveHeadToHeadingPosition() {
  int startVal = servoPositions[currentHeadPosition];
  int finalVal = servoPositions[heading] - 1;
  if (startVal > finalVal) {
    startVal = startVal * -1;
    finalVal = finalVal * -1;
  }
  headServo.write(abs(finalVal));
  delay(HEAD_MOVEMENT_DELAY * abs(finalVal - startVal));
  currentHeadPosition = heading;
}

// Set "heeading" variable to next position and moves head
void moveHeadToNextPosition() {
  if (currentHeadPosition == 2) {
    direction = -1;
  } else if (currentHeadPosition == 0) {
    direction = 1;
  }
  heading = currentHeadPosition + direction;
  moveHeadToHeadingPosition();
}

// Returns true if an obstacle is in "dist" distance
boolean isObstacleIn(int dist) {
  return !(distanceCm >= dist || distanceCm == 0);
}

// Turn motors ON at "sSpeed" speed and sleeps for "time" in milliseconds
void activateMotors(int mSpeed, int time) {
  analogWrite(ENA, mSpeed);
  analogWrite(ENB, mSpeed - ((mSpeed * LEFT_WHEEL_DIFF) / MAX_SPEED));
  delay(time);
}

// Set the correct values for each input to the motor controller
void setMotorValues(int values[]) {
  digitalWrite(IN1, values[0]);
  digitalWrite(IN2, values[1]);
  digitalWrite(IN3, values[2]);
  digitalWrite(IN4, values[3]);
}

// Stop both motors
void stopMotors() {
  setMotorValues(motorModes[0]);
}

// Set motors direction to move Left
void dLeft() {
  setMotorValues(motorModes[1]);
}

// Set motors direction to move Right
void dRight() {
  setMotorValues(motorModes[2]);
}

// Set motors direction to move Forward
void dFwd() {
  setMotorValues(motorModes[3]);
}

// Set motors direction to move Back
void dBack() {
  setMotorValues(motorModes[4]);
}

void turnBotToHeadPosition() {
  switch (currentHeadPosition) {
  case 0:
    dLeft();
    break;
  case 1:
    dFwd();
    break;
  case 2:
    dRight();
    break;
  }
}

// Reset head to be looking at the front
void resetHead() {
  heading = 1;
  moveHeadToHeadingPosition();
  direction = direction * -1;
}

// Process BT data that's active in both modes
void processBluetoothGeneralData() {
  switch (btData) {
  case 'm':
    manualMode = !manualMode;
    resetHead();
    stopMotors();
    delay(1000);
    break;
  case 'x':
    motorSpeed = MAX_SPEED / 2;
    activateMotors(motorSpeed, 0);
    break;
  case 'f':
    motorSpeed = MAX_SPEED;
    activateMotors(motorSpeed, 0);
    break;
  }
}

// Process "Manual mode only" data
void processBluetoothManualData() {
  switch (btData) {
  case 'z':
    stopMotors();
    break;
  case 'l':
    dLeft();
    activateMotors(motorSpeed, 0);
    break;
  case 'u':
    dFwd();
    activateMotors(motorSpeed, 0);
    break;
  case 'r':
    dRight();
    activateMotors(motorSpeed, 0);
    break;
  case 'd':
    dBack();
    activateMotors(motorSpeed, 0);
    break;
  case 'g':
    digitalWrite(LED_PIN, ledOn ? LOW : HIGH);
    ledOn = !ledOn;
    break;
  case 's':
    moveHeadToNextPosition();
    break;
  }
}

void loop() {

  // Check if new data was sent via Bluetooth
  if (Serial.available() > 0) {
    btData = Serial.read();
    processBluetoothGeneralData();
  }

  if (manualMode) {
    processBluetoothManualData();
    btData = 0; // Clear already processed data
  } else {
    // Check if there is an obstacle
    if (isObstacleIn(MIN_DISTANCE)) {
      //If there is an obstacle stop the robot and move the head
      contMovement = false;
      stopMotors();
      moveHeadToNextPosition();
    } else if (contMovement == false) {
      // if there is no obstacle but the robot was stopped
      // rotate it to the head position
      turnBotToHeadPosition();
      resetHead();
      activateMotors(MAX_SPEED, 45);
      stopMotors();
      // Continue moving as normal
      contMovement = true;
      delay(1000);
    } else {
      // if there is no obstacle and the robot should continue moving
      // moves forward
      dFwd();
      activateMotors(motorSpeed, 0);
    }

    // Check for obstacles
    readSensor();
  }
}

