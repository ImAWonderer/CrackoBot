#include <NewPing.h>
#include <Servo.h> 

// Utrasonic Sensor
#define TRIGGER_PIN         12
#define ECHO_PIN            11
#define SONIC_READ_DELAY    50

// Servo
#define SERVO_PIN           9
#define HEAD_MOVEMENT_DELAY 5

// Motor Contoller
#define IN1                 7
#define IN2                 8
#define ENA                 6
#define IN3                 5  
#define IN4                 4   
#define ENB                 3

// General Variables
#define MAX_DISTANCE        200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define MIN_DISTANCE        30 
#define MAX_SPEED           100
#define LEFT_WHEEL_DIFF     22  // Percentage of diff from the motors speed

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
Servo headServo;

boolean contMovement = true;
boolean manualMode = true;

int motorSpeed = MAX_SPEED;
int valPos[3] = { 180, 90, 1 }; 
int distanceCm = 0;
int currentHeadPosition = 0;
int heading = 1;
int direction = 1;
char btData = 0;

void setup() {
  Serial.begin(9600);
  headServo.attach(SERVO_PIN);
  pinMode (ENA, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (ENB, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
}

void readSensor() {
  delay(SONIC_READ_DELAY);
  unsigned int sonarRead = sonar.ping(); 
  distanceCm = sonarRead / US_ROUNDTRIP_CM; 
}

void moveHead(int start, int final) {
  int startVal = start;
  int finalVal = final - 1;
  if (startVal > finalVal) {
    startVal = startVal * -1;
    finalVal = finalVal * -1;
  }
  headServo.write(abs(finalVal));           
  delay(HEAD_MOVEMENT_DELAY* abs(finalVal-startVal));                    
  currentHeadPosition = heading;
}

void updateHeadingPosition() {
  if (currentHeadPosition == 2) {
    direction = -1;
  } else if (currentHeadPosition == 0) {
    direction = 1;
  }
  heading = currentHeadPosition + direction;
}

boolean isObstacleIn(int dist) {
  return !(distanceCm >= dist || distanceCm == 0);
}

void activateMotors(int mSpeed, int time) {
  analogWrite(ENA, mSpeed);
  analogWrite(ENB, mSpeed - ((mSpeed * LEFT_WHEEL_DIFF) / MAX_SPEED));
  delay(time);
}

void stopMotors() {
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, LOW);
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, LOW);
}

void dLeft() {
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);
}

void dRight() {
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
}

void dFwd() {
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
}

void dBack() {
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);
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

void resetHead() {
  heading = 1;
  moveHead(valPos[currentHeadPosition], valPos[heading]);
}

void loop() {
  if(Serial.available() > 0) {
    btData = Serial.read(); 
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
  if(manualMode) {
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
        heading = 0;
        moveHead(valPos[currentHeadPosition], valPos[heading]);
        break; 
      case 's': 
        heading = 2;
        moveHead(valPos[currentHeadPosition], valPos[heading]);
        break;       
    }
  } else {
    if (isObstacleIn(MIN_DISTANCE)) {
      contMovement = false;
      stopMotors();
      updateHeadingPosition();
      moveHead(valPos[currentHeadPosition], valPos[heading]);
    } else {

      if (contMovement == false) {

        turnBotToHeadPosition();

        resetHead();
        
        activateMotors(MAX_SPEED, 45);

        stopMotors();
        contMovement = true;

        delay(1000);
      } else {
        dFwd();
        activateMotors(motorSpeed, 0);
      }
    }
    readSensor();
  }
}

