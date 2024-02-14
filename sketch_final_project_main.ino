/*
Names: Zach Rudder, John D'Ambrosio, Cedric Hollande
Team 23
MEAM 5100
Final Project
*/
#include <WiFi.h>
#include <WiFiUdp.h>
#include "webpage.h"
#include "html510.h"
#include "PIDcontoller.h"
#include <Wire.h>
#include <VL53L0X.h>
#include <ESP32Servo.h>
#include "vive510.h"
/**************************************************************/
// Motor and Encoder Definitions
#define L_MOTOR_PWM_PIN 21
#define L_MOTOR_FORWARD_PIN 20
#define L_MOTOR_BACKWARD_PIN 19
#define R_MOTOR_PWM_PIN 42
#define R_MOTOR_FORWARD_PIN 41
#define R_MOTOR_BACKWARD_PIN 40

#define L_ENCODER_A_PIN 35
#define L_ENCODER_B_PIN 36
#define R_ENCODER_A_PIN 38
#define R_ENCODER_B_PIN 39
/**************************************************************/
// Set up the state machine for the tasks
enum RobotMode { IDLE, WALL_FOLLOWING, BEACON_TRACKING, POLICE_TRACKING };
RobotMode currentMode = IDLE;
/**************************************************************/
// Set up TOF sensors

// Define the XSHUT pins for each sensor
const int XSHUT_front = 2;  // Pin connected to the XSHUT of the front sensor
const int XSHUT_right = 3;  // Pin connected to the XSHUT of the right sensor

// Initialize our TOF sensors
VL53L0X frontSensor;
VL53L0X rightSensor;
/**************************************************************/
// Intitializations and definitions for webpage and UDP

// Initialize the web server on port 80
HTML510Server h(80);
WiFiUDP UDPServer;
#define UDPPORT 2510  // port for game obj transmission
/**************************************************************/
// PID AND MOTOR CONTROL

// Motor Control Variables
const int L_PWM_CHANNEL = 0;
const int R_PWM_CHANNEL = 1;
const int PWM_RESOLUTION_BITS = 11;  // 11-bit resolution for 60 Hz PWM
const int PWM_FREQUENCY_HZ = 30;

// Motor Encoder Variables
volatile long leftEncoderPosition = 0;
volatile long rightEncoderPosition = 0;
volatile bool lastALeft = LOW;
volatile bool lastBLeft = LOW;
volatile bool lastARight = LOW;
volatile bool lastBRight = LOW;
const float countsPerRevolution = 1920;  // Counts per revolution of the gearbox output

unsigned long lastTime = 0;

// PID for Speed Control
const float leftTargetRPM = -45;
const float rightTargetRPM = -45;
PIDController pidLeft(4.0, 0.1, 0, -2047, 2047);
PIDController pidRight(4.0, 0.1, 0, -2047, 2047);  // Assuming same PID parameters for right motor

// PID for Wall Following
PIDController pidWall(0.1, 0.01, 0, -2047, 2047);  // Adjust these values to change output
const float targetDistance = 200.0;              // Target distance from the wall in mm
/**************************************************************/
// WiFi credentials and IP configuration
const char *ssid = "TP-Link_E0C8";
const char *password = "52665134";
IPAddress myIp(192, 168, 1, 117);
IPAddress ipTarget(192, 168, 1, 255);  // 255 => broadcast
/**************************************************************/
// VIVE SETUP
#define RGBLED 18          // for ESP32S2 Devkit pin 18
#define FRONT_VIVE_PIN 13  // pin receiving signal from front vive
#define REAR_VIVE_PIN 12   // pin receiving signal from rear vive
#define UDPPORT 2510       // For GTA 2023 game
#define STUDENTIP 117      // A teammembers assigned IP number
#define teamNumber 23
#define FREQ 1  // in Hz
#define CENTER_X_COORD 4000
#define CENTER_Y_COORD 4000

// Initialize the coordinate and angle variables
int frontX, frontY, rearX, rearY;
float xRobot, yRobot, xPolice, yPolice;
float robotAngle, angleToTarget, angleDifference, angle1, angle2;

// Initialize the two sensors
Vive510 frontVive(FRONT_VIVE_PIN);
Vive510 rearVive(REAR_VIVE_PIN);

// Setup the filters that will be used
const int FILTER_SIZE = 5;
float frontXFilter[FILTER_SIZE] = { 0 };
float frontYFilter[FILTER_SIZE] = { 0 };
float rearXFilter[FILTER_SIZE] = { 0 };
float rearYFilter[FILTER_SIZE] = { 0 };
float policeXFilter[FILTER_SIZE] = { 0 };
float policeYFilter[FILTER_SIZE] = { 0 };
int policeXFilterIndex = 0;
int policeYFilterIndex = 0;
int filterIndex = 0;
/**************************************************************/
//FREQUENCY DETECTION

//Set up variables and pin to be used for frequency detection
volatile uint32_t risingEdge1 = 0;
volatile uint32_t risingEdge2 = 0;
volatile bool edgeCaptured = false;
int numSweeps = 0;
int customDelay;

#define PHOTOTRANSISTOR_PIN1 4  // Left phototransistor
#define PHOTOTRANSISTOR_PIN2 6  // Right phototransistor
#define SERVO_PIN 10            // Beacon tracking servo
int PHOTOTRANSISTOR_PIN;
float targetFreq = 23.0;

// Create the servo object
Servo beaconServo;

// Constants for servo positions
const int SERVO_MIN_POS = 0;
const int SERVO_MAX_POS = 180;
int SERVO_SPEED = 3;  // Adjust for faster or slower servo movement
int servoPos = 90;    // Start from the middle position

bool beaconLeftDetected = false;
bool beaconRightDetected = false;
/**************************************************************/
// Logic variables
bool motorsPowered = false;
bool wallFollowingActive = false;
bool beaconTrackingActive = false;
bool locatePoliceActive = false;
bool stoppedPressed = false;
/**************************************************************/
// TIMER FOR UDP -- not used during graded evaluation 

// Stop button is attached to onboard button PIN 3
#define BTN_STOP_ALARM    3

hw_timer_t * timer = NULL;

volatile uint32_t isrCounter = 0;
/**************************************************************/

/**************************************************************/
// Functions related to the UDP sending timer

// Interrupt to send the UDP message with the robots coordinates
void IRAM_ATTR onTimer(){
  UdpSend(xRobot, yRobot);
}

// Function to set up the 1 Hz timer
void setupTimer(){
  
  Serial.println("timers start setup");
  // Use 1st timer of 2 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer. 3rd param currently must be false
  timerAttachInterrupt(timer, &onTimer, false);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (3rd parameter)
  timerAlarmWrite(timer, 1000000, true);

  // Start an alarm
  timerAlarmEnable(timer);
  Serial.println("timers setup");
}
/**************************************************************/

/**************************************************************/
// Functions related to Vive

// Function to update a particular filter
void updateFilter(float *filter, int filterIndex, float newValue) {
  filter[filterIndex] = newValue;
  filterIndex = (filterIndex + 1) % FILTER_SIZE;
}

// Function to return the average value
float getFilteredValue(float *filter) {
  float sum = 0;
  for (int i = 0; i < FILTER_SIZE; ++i) {
    sum += filter[i];
  }
  return sum / ((float) FILTER_SIZE);
}

// Function to read sensor values and update filters
void readAndFilterSensors(Vive510 &viveSensor, float *xFilter, float *yFilter, int filterIndex) {
  if (viveSensor.status() == VIVE_RECEIVING) {
    // Update the filters with the current readings
    updateFilter(xFilter, filterIndex, viveSensor.xCoord());
    updateFilter(yFilter, filterIndex, viveSensor.yCoord());
  } else {
    // If no reading is received, attempt to resynchronize
    viveSensor.sync(5);
  }
}

// Function to calculate the centroid of the robot
void calculateCentroid(float frontX, float frontY, float rearX, float rearY) {
  // Calculate the centroid based on filtered sensor values
  xRobot = (frontX + rearX) / 2;
  yRobot = (frontY + rearY) / 2;
}

// Function to normalize the angle if it is not within 0-2PI
float normalizeAngle(float angle) {
  while (angle < 0) {
    angle += 2 * PI;
  }
  while (angle >= 2 * PI) {
    angle -= 2 * PI;
  }
  return angle;
}

// Function to send our robot's X and Y coords
void UdpSend(int x, int y) {
  char udpBuffer[20];
  sprintf(udpBuffer, "%02d: %4d, %4d", teamNumber, x, y);

  UDPServer.beginPacket(ipTarget, UDPPORT);
  UDPServer.println(udpBuffer);
  UDPServer.endPacket();
  Serial.println(udpBuffer);
}

// Function to handle incoming UDP packets -- how we get the police car's coords
void handleUDPServer() {
  const int UDP_PACKET_SIZE = 14;  // can be up to 65535          
  uint8_t packetBuffer[UDP_PACKET_SIZE];

  int cb = UDPServer.parsePacket();  // if there is no message cb=0
  if (cb) {
    int xSender, ySender, senderTeam;
    packetBuffer[13] = 0;  // null terminate string

    UDPServer.read(packetBuffer, UDP_PACKET_SIZE);
    xSender = atoi((char *)packetBuffer + 3);  // ##,####,#### 2nd indexed char
    ySender = atoi((char *)packetBuffer + 8);  // ##,####,#### 7th indexed char
    senderTeam = atoi((char *)packetBuffer);
    if (senderTeam == 0) {
      // Update filters for the police car's coordinates
      updateFilter(policeXFilter, policeXFilterIndex, (float)xSender);
      updateFilter(policeYFilter, policeYFilterIndex, (float)ySender);

      // Get the filtered values
      xPolice = xSender;//getFilteredValue(policeXFilter);
      yPolice = ySender;//getFilteredValue(policeYFilter);
      Serial.print("Police car X: "); Serial.println(xPolice);
      Serial.print("Police car Y: "); Serial.println(yPolice);
    }
  }
}

// Function that gets the coordinates and calculate the angle needed to rotate
void runVive() {

  static uint16_t x1, y1;
  static uint16_t x2, y2;

  readAndFilterSensors(frontVive, frontXFilter, frontYFilter, filterIndex);
  readAndFilterSensors(rearVive, rearXFilter, rearYFilter, filterIndex);

  // After updating filters, retrieve the filtered values
  float frontX = getFilteredValue(frontXFilter);
  float frontY = getFilteredValue(frontYFilter);
  float rearX = getFilteredValue(rearXFilter);
  float rearY = getFilteredValue(rearYFilter);

  // Print statements used for debugging
  Serial.print("Front Vive working: X coord: ");
  Serial.print(frontX);
  Serial.print(" Y coord: ");
  Serial.println(frontY);
  Serial.print("Rear Vive working: X coord: ");
  Serial.print(rearX);
  Serial.print(" Y coord: ");
  Serial.println(rearY);
  Serial.print("Police car X: "); Serial.println(xPolice);
  Serial.print("Police car Y: "); Serial.println(yPolice);


  // Only calculate the robot's position and angle if both trackers have valid coordinates
  if (frontX != 0 && frontX < 60000 && frontY != 0 && frontY < 60000 && rearX != 0 && rearX < 60000 && rearY != 0 && rearY < 60000) {
    calculateCentroid(frontX, frontY, rearX, rearY);
    robotAngle = atan2(rearY - frontY, rearX - frontX);  // calculate robots orientation
    robotAngle = normalizeAngle(robotAngle);
    angleToTarget = atan2(yRobot - yPolice, xRobot - xPolice);  //calculate angle to target
    angleToTarget = normalizeAngle(angleToTarget);

    // Print statements used for debugging
    Serial.print("Robot angle: ");
    Serial.println(robotAngle);
    Serial.print("Angle to police: ");
    Serial.println(angleToTarget);

    angleDifference = angleToTarget - robotAngle;
    // Normalize the angle difference to be within -PI and PI
    if (angleDifference > PI) {
      angleDifference -= 2 * PI;
    } else if (angleDifference < -PI) {
      angleDifference += 2 * PI;
    }
  }
}

// Function to push the police car once the robot navigates to it
void pushPolice(){
  float distanceFront = frontSensor.readRangeContinuousMillimeters();

  if (distanceFront < 25 && !stoppedPressed){
    // Ram the police car
    digitalWrite(L_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(L_MOTOR_BACKWARD_PIN, HIGH);
    digitalWrite(R_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(R_MOTOR_BACKWARD_PIN, HIGH);
    ledcWrite(L_PWM_CHANNEL, 2000);
    ledcWrite(R_PWM_CHANNEL, 2000);
    delay(10000);
    // Ram the police car 3 times to push it far enough
    int i;
    for (i = 0; i < 3; i++ ){
      // Reverse
      digitalWrite(L_MOTOR_FORWARD_PIN, HIGH);
      digitalWrite(L_MOTOR_BACKWARD_PIN, LOW);
      digitalWrite(R_MOTOR_FORWARD_PIN, HIGH);
      digitalWrite(R_MOTOR_BACKWARD_PIN, LOW);
      ledcWrite(L_PWM_CHANNEL, 500);
      ledcWrite(R_PWM_CHANNEL, 500);
      delay(500);
      // Ram the police car again
      digitalWrite(L_MOTOR_FORWARD_PIN, LOW);
      digitalWrite(L_MOTOR_BACKWARD_PIN, HIGH);
      digitalWrite(R_MOTOR_FORWARD_PIN, LOW);
      digitalWrite(R_MOTOR_BACKWARD_PIN, HIGH);
      ledcWrite(L_PWM_CHANNEL, 2000);
      ledcWrite(R_PWM_CHANNEL, 2000);
      delay(10000);
    }
  }
}

// Function to rotate the police car based on the difference in angles
void rotateToPolice(float error) {
  // Print statements used for debugging
  Serial.print("Average angle error: ");
  Serial.println(error);

  // If the error is positive and not within our tolerance, rotate right
  if (fabs(error) > .25 && error < 0) {
    // Rotate to the right slightly
    digitalWrite(L_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(L_MOTOR_BACKWARD_PIN, HIGH);
    digitalWrite(R_MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(R_MOTOR_BACKWARD_PIN, LOW);
    ledcWrite(L_PWM_CHANNEL, 300);
    ledcWrite(R_PWM_CHANNEL, 300);
    delay(75);
    // Stop the motors
    digitalWrite(L_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(L_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(R_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(R_MOTOR_BACKWARD_PIN, LOW);
  } 
  // If the error is negative, rotate left
  else if (fabs(error) > .25 && error > 0) {
    // Rotate to the left slightly
    digitalWrite(L_MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(L_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(R_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(R_MOTOR_BACKWARD_PIN, HIGH);
    ledcWrite(L_PWM_CHANNEL, 300);
    ledcWrite(R_PWM_CHANNEL, 300);
    delay(75);
    // Stop the motors
    digitalWrite(L_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(L_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(R_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(R_MOTOR_BACKWARD_PIN, LOW);
  } 
  // if the robot is lined up within the tolerance of the police car
  else {
    // Stop robot
    digitalWrite(L_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(L_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(R_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(R_MOTOR_BACKWARD_PIN, LOW);
  }
}

// Function to move the robot to the car once it is aligned with it
void moveUponDetection(float error) {
  if (fabs(error) < .25) {
    digitalWrite(L_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(L_MOTOR_BACKWARD_PIN, HIGH);
    digitalWrite(R_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(R_MOTOR_BACKWARD_PIN, HIGH);
    unsigned long int moveStartTime = millis();
    unsigned long int moveDuration = 1500;
    while (millis() - moveStartTime < moveDuration) {
      float distanceFront = frontSensor.readRangeContinuousMillimeters();

      // If the robot is about to hit the police car (or a wall)
      if (distanceFront < 25) {
        digitalWrite(L_MOTOR_FORWARD_PIN, LOW);
        digitalWrite(L_MOTOR_BACKWARD_PIN, LOW);
        digitalWrite(R_MOTOR_FORWARD_PIN, LOW);
        digitalWrite(R_MOTOR_BACKWARD_PIN, LOW);
      }
      // Use the PID controller to maintain the target speed
      runPIDControl(false);
  }
    // If the robot is about to hit a wall, stop it and reverse slightly
    float distanceFront = frontSensor.readRangeContinuousMillimeters();
    if (distanceFront < 25){
      digitalWrite(L_MOTOR_FORWARD_PIN, HIGH);
      digitalWrite(L_MOTOR_BACKWARD_PIN, LOW);
      digitalWrite(R_MOTOR_FORWARD_PIN, HIGH);
      digitalWrite(R_MOTOR_BACKWARD_PIN, LOW);
      delay(200);
    }
    // Stop the motors
    digitalWrite(L_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(L_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(R_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(R_MOTOR_BACKWARD_PIN, LOW);
  }
}

// Function to check if the robot actually reached the police car and is facing it
void checkProximity(){
  float distanceFront = frontSensor.readRangeContinuousMillimeters();

  // If the robot is within 10 coords of the police
  if (abs(frontX - xPolice) < 10 && abs(frontY - yPolice) < 10){
    // If the front sensor is detecting the robot, meaning the robot is facing the police
    if (distanceFront < 25){
      digitalWrite(L_MOTOR_FORWARD_PIN, LOW);
      digitalWrite(L_MOTOR_BACKWARD_PIN, HIGH);
      digitalWrite(R_MOTOR_FORWARD_PIN, LOW);
      digitalWrite(R_MOTOR_BACKWARD_PIN, HIGH);
      ledcWrite(L_PWM_CHANNEL, 2000);
      ledcWrite(R_PWM_CHANNEL, 2000);
      delay(10000);
    } else if (distanceFront < 50){
      digitalWrite(L_MOTOR_FORWARD_PIN, LOW);
      digitalWrite(L_MOTOR_BACKWARD_PIN, HIGH);
      digitalWrite(R_MOTOR_FORWARD_PIN, LOW);
      digitalWrite(R_MOTOR_BACKWARD_PIN, HIGH);
      ledcWrite(L_PWM_CHANNEL, 600);
      ledcWrite(R_PWM_CHANNEL, 600);
      delay(2000);
    }
  }
}

// Function to run all the functions in order to locate and move to the police car
void runLocatePoliceCar(){
  handleUDPServer();
  runVive();
  rotateToPolice(angleDifference);
  moveUponDetection(angleDifference);
  checkProximity();
  pushPolice();
}
/**************************************************************/

/**************************************************************/
// Functions related to beacon tracking -- not used for graded evaluation

// Interrupt service routine for frequency detection
void IRAM_ATTR onRisingEdge() {
  if (!risingEdge1) {
    risingEdge1 = micros();
  } else if (!risingEdge2) {
    risingEdge2 = micros();
    edgeCaptured = true;
  }
}

// Function that measures the frequency picked up by the phototransistors
float measureFreq(int pinNumber) {

  if (pinNumber == 1) {
    PHOTOTRANSISTOR_PIN = PHOTOTRANSISTOR_PIN1;
  } else if (pinNumber == 2) {
    PHOTOTRANSISTOR_PIN = PHOTOTRANSISTOR_PIN2;
  }

  risingEdge1 = 0;
  risingEdge2 = 0;
  edgeCaptured = false;

  attachInterrupt(digitalPinToInterrupt(PHOTOTRANSISTOR_PIN), onRisingEdge, RISING);

  delay(10);
  if (!edgeCaptured) return -1;  // Wait for two rising edges to be captured

  detachInterrupt(digitalPinToInterrupt(PHOTOTRANSISTOR_PIN));

  uint32_t period = risingEdge2 - risingEdge1;
  float frequency = 0;
  if (period > 0) {
    frequency = 1000000.0 / period;  // Convert period in microseconds to frequency
  }
  return frequency;
}

// Function to update the servo position
void updateServo(int pinNumber) {

  // If the beacon is on the left, move the servo one degree to the left
  if (pinNumber == 1) {
    if (SERVO_SPEED > 0) {
      servoPos += SERVO_SPEED;
    } else {
      servoPos -= SERVO_SPEED;
    }
  }
  // If the beacon is on the right, move the servo one degree to the right
  else if (pinNumber == 2) {
    if (SERVO_SPEED < 0) {
      servoPos += SERVO_SPEED;
    } else {
      servoPos -= SERVO_SPEED;
    }
  }

  beaconServo.write(servoPos);
  delay(15);
}

// Function to sweep servo back and forth
void sweepServo() {

  if (numSweeps > 2) {
    turnLeft(200);
    numSweeps = 0;
  }

  // Increment or decrement the servo position
  servoPos += SERVO_SPEED;
  if (servoPos >= SERVO_MAX_POS || servoPos <= SERVO_MIN_POS) {
    SERVO_SPEED = -SERVO_SPEED;  // Change direction
    numSweeps++;
  }

  beaconServo.write(servoPos);
  delay(15);
}

// Function to determine if a target frequency has been detected
bool frequencyDetected(float freq) {

  if (freq >= targetFreq - 10 && freq <= targetFreq + 10) {
    return true;
  } else {
    return false;
  }
}

// Function to start beacon detection
void runBeaconDetection() {

  float freqL = measureFreq(1);
  float freqR = measureFreq(2);

  beaconLeftDetected = frequencyDetected(freqL);
  beaconRightDetected = frequencyDetected(freqR);

  if (beaconLeftDetected && beaconRightDetected) {
    moveForward();
    customDelay = millis();
    while (millis() - customDelay < 2000){
      runPIDControl(false);
    }
  } else if (beaconLeftDetected && !beaconRightDetected) {
    Serial.println("Target detected on left. Turn left.");
    updateServo(1);
    turnLeft(50);
  } else if (!beaconLeftDetected && beaconRightDetected) {
    Serial.println("Target detected on right. Turn right.");
    updateServo(2);
    turnRight(50);
  } else if (!beaconLeftDetected && !beaconRightDetected) {
    Serial.println("No target detected. Continue rotating.");
    sweepServo();
  }
}
/**************************************************************/

/**************************************************************/
// Functions to handle when buttons on the webpage are pressed

// Function to handle when the left turn button is pressed
void handleButtonLeft() {
  turnLeft(350);
  stoppedPressed = false;
}

// Function to handle when the right turn button is pressed
void handleButtonRight() {
  turnRight(350);
  stoppedPressed = false;
}

// Function to handle when the forwards button is pressed
void handleButtonForward() {
  moveForward();
  stoppedPressed = false;
}

// Function to handle when the backwards button is pressed
void handleButtonBackward() {
  moveBackward();
  stoppedPressed = false;
}

// Function that loads the webpage
void handleRoot() {
  h.sendhtml(body);
}

// Function to handle when the stop button is pressed
void handleButtonStop() {

  wallFollowingActive = false;
  beaconTrackingActive = false;
  locatePoliceActive = false;
  currentMode = IDLE;
  stoppedPressed = true;
  fullStop();
}

// Function to handle when the beacon tracking button is pressed
void handleButtonBeaconTracking() {
  beaconTrackingActive = true;
  currentMode = BEACON_TRACKING;
  stoppedPressed = false;
}

// Function to handle when the wall following button is pressed
void handleButtonWallFollowing() {
  wallFollowingActive = true;  
  currentMode = WALL_FOLLOWING;
  stoppedPressed = false;
}

// Function to handle when the police car tracking button is pressed
void handleButtonPolice() {
  locatePoliceActive = true;
  currentMode = POLICE_TRACKING;
  stoppedPressed = false;
}

// Function to "start" motors
void handleButtonMotorPower() {
  motorsPowered = true;
  stoppedPressed = false;
}
/**************************************************************/

/**************************************************************/
// Functions for basic movements of the robot

// Function to move the robot backwards
void moveBackward() {

  fullStop();
  digitalWrite(L_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(L_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(R_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(R_MOTOR_BACKWARD_PIN, LOW);
  runPIDControl(false);
}

// Function that sets the motors to move the robot forward
void moveForward() {

  fullStop();
  digitalWrite(L_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(L_MOTOR_BACKWARD_PIN, HIGH);  // Motors are set up so backwards moves the robot forwards
  digitalWrite(R_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(R_MOTOR_BACKWARD_PIN, HIGH);
  runPIDControl(false);
}

// Function that completely stops the robot
void fullStop() {

  digitalWrite(L_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(L_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(R_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(R_MOTOR_BACKWARD_PIN, LOW);
}

// Function to turn the robot left 
void turnLeft(int time) {

  fullStop();
  digitalWrite(L_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(L_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(R_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(R_MOTOR_BACKWARD_PIN, HIGH);
  delay(time);
  fullStop();
}

// Function to turn the robot right 
void turnRight(int time) {

  fullStop();
  digitalWrite(L_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(L_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(R_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(R_MOTOR_BACKWARD_PIN, LOW);
  delay(time);
  fullStop();
}
/**************************************************************/

/**************************************************************/
// Functions for the motor encoders

void IRAM_ATTR handleLeftEncoderA() {
  bool A = digitalRead(L_ENCODER_A_PIN);
  bool B = digitalRead(L_ENCODER_B_PIN);
  if (A != lastALeft) {
    lastALeft = A;
    if (A == B) {
      leftEncoderPosition++;
    } else {
      leftEncoderPosition--;
    }
  }
}

void IRAM_ATTR handleLeftEncoderB() {
  bool A = digitalRead(L_ENCODER_A_PIN);
  bool B = digitalRead(L_ENCODER_B_PIN);
  if (B != lastBLeft) {
    lastBLeft = B;
    if (A != B) {
      leftEncoderPosition++;
    } else {
      leftEncoderPosition--;
    }
  }
}

void IRAM_ATTR handleRightEncoderA() {
  bool A = digitalRead(R_ENCODER_A_PIN);
  bool B = digitalRead(R_ENCODER_B_PIN);
  if (A != lastARight) {
    lastARight = A;
    if (A == B) {
      rightEncoderPosition++;
    } else {
      rightEncoderPosition--;
    }
  }
}

void IRAM_ATTR handleRightEncoderB() {
  bool A = digitalRead(R_ENCODER_A_PIN);
  bool B = digitalRead(R_ENCODER_B_PIN);
  if (B != lastBRight) {
    lastBRight = B;
    if (A != B) {
      rightEncoderPosition++;
    } else {
      rightEncoderPosition--;
    }
  }
}
/**************************************************************/

/**************************************************************/
// Functions for wall following and general PID 

// Function that put the robot in wall following mode
void performWallFollowing() {

  if (!stoppedPressed){
    // Set up motors to go foward
    digitalWrite(L_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(L_MOTOR_BACKWARD_PIN, HIGH);
    digitalWrite(R_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(R_MOTOR_BACKWARD_PIN, HIGH);
    runPIDControl(true);
  }
  
}

// Function that run our PID control for the motors
void runPIDControl(bool withWallFollowing) {

  float distanceFront = frontSensor.readRangeContinuousMillimeters();

  // Check if there is an obstacle in front of the robot
  if (!frontSensor.timeoutOccurred() && distanceFront <= 300 && motorsPowered) {
    // Stop the motors
    fullStop();

    // Turn the robot
    digitalWrite(L_MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(R_MOTOR_BACKWARD_PIN, HIGH);
    ledcWrite(L_PWM_CHANNEL, 300);
    ledcWrite(R_PWM_CHANNEL, 300);
    delay(70);  // Adjust delay as needed for the turn

    // Stop the motors again
    fullStop();

    // Set up motors to go forward
    digitalWrite(L_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(L_MOTOR_BACKWARD_PIN, HIGH);
    digitalWrite(R_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(R_MOTOR_BACKWARD_PIN, HIGH);

    // Now, the robot should be oriented away from the obstacle
    // Continue to wall following logic
  }

  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 10) {
    long leftEncoderCounts = leftEncoderPosition;
    long rightEncoderCounts = rightEncoderPosition;
    leftEncoderPosition = 0;
    rightEncoderPosition = 0;

    // Calculate left and right RPMs and distance to the wall
    float measurementIntervalInSeconds = (currentTime - lastTime) / 1000.0;
    float leftRPM = (leftEncoderCounts / countsPerRevolution) * (60 / measurementIntervalInSeconds);
    float rightRPM = (rightEncoderCounts / countsPerRevolution) * (60 / measurementIntervalInSeconds);
    float distanceRight = rightSensor.readRangeContinuousMillimeters();

    // Calculate the control signals for the motors
    float uL = pidLeft.compute(leftTargetRPM, leftRPM, measurementIntervalInSeconds);
    float uR = pidRight.compute(rightTargetRPM, rightRPM, measurementIntervalInSeconds);  // Assuming right motor needs similar control

    int pwmL;
    int pwmR;

    if (rightSensor.timeoutOccurred()) {
      Serial.println("rightSensor timeout!");
    } else if (frontSensor.timeoutOccurred()) {
      Serial.println("frontSensor timeout!");
    } else {
      // Calculate the control signal due to the distance from the wall
      float wallOutput = pidWall.compute(targetDistance, distanceRight, measurementIntervalInSeconds);

      // Only apply the wallOutput if the robot is not in the deadband range and withWallFollowing is true
      if (abs(distanceRight - targetDistance) > 25 && withWallFollowing) {
        pwmL = abs(uL) - wallOutput;
        pwmR = abs(uR) + wallOutput;
      } else {
        pwmL = abs(uL);
        pwmR = abs(uR);
      }
    }

    ledcWrite(L_PWM_CHANNEL, pwmL);
    ledcWrite(R_PWM_CHANNEL, pwmR);

    lastTime = currentTime;
  }
}
/**************************************************************/

/**************************************************************/
// Functions that setup all the various components

// Function to setup the TOF sensors
void setupTOFSensors() {

  Wire.begin();

  // Initialize front sensor
  pinMode(XSHUT_front, OUTPUT);
  digitalWrite(XSHUT_front, LOW);
  delay(10);
  pinMode(XSHUT_front, INPUT);
  delay(10);
  frontSensor.init();
  frontSensor.setAddress(0x30);  // Unique address for the front sensor
  frontSensor.startContinuous();

  // Initialize right sensor
  pinMode(XSHUT_right, OUTPUT);
  digitalWrite(XSHUT_right, LOW);
  delay(10);
  pinMode(XSHUT_right, INPUT);
  delay(10);
  rightSensor.init();
  rightSensor.setAddress(0x31);  // Unique address for the right sensor
  rightSensor.startContinuous();

  // Verify addresses
  Serial.print("Front Sensor Address: 0x");
  Serial.println(frontSensor.getAddress(), HEX);
  Serial.print("Right Sensor Address: 0x");
  Serial.println(rightSensor.getAddress(), HEX);
}

// Function to set up WiFI
void setupWiFi() {

  WiFi.begin(ssid, password);
  WiFi.config(myIp,                          // Device IP address
              IPAddress(192, 168, 1, 1),     // gateway (not important for 5100)
              IPAddress(255, 255, 255, 0));  // net mask

  UDPServer.begin(UDPPORT);  // 2510 forgame  arbitrary UDP port# need to use same one
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.printf("WiFi connected to %s\n", ssid);
  Serial.print("Using static IP ");
  Serial.print(myIp);
  Serial.print(" and UDP port ");
  Serial.println(UDPPORT);
}

// Function to set up the handler routines
void setupHandlerRoutines() {

  h.begin();
  h.attachHandler("/", handleRoot);
  h.attachHandler("/hitLeft", handleButtonLeft);
  h.attachHandler("/hitRight", handleButtonRight);
  h.attachHandler("/hitStop", handleButtonStop);
  h.attachHandler("/hitForward", handleButtonForward);
  h.attachHandler("/hitBack", handleButtonBackward);
  h.attachHandler("/hitWallFollowing", handleButtonWallFollowing);
  h.attachHandler("/hitPoliceCar", handleButtonPolice);
  h.attachHandler("/hitTrackBeacon", handleButtonBeaconTracking);
  h.attachHandler("/hitMotorPower", handleButtonMotorPower);
}

// Function to set up motor pins, encoder pins, and PWM
void setupPins() {

  pinMode(L_MOTOR_PWM_PIN, OUTPUT);
  pinMode(L_MOTOR_BACKWARD_PIN, OUTPUT);
  pinMode(L_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(R_MOTOR_PWM_PIN, OUTPUT);
  pinMode(R_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(R_MOTOR_BACKWARD_PIN, OUTPUT);

  pinMode(L_ENCODER_A_PIN, INPUT);
  pinMode(L_ENCODER_B_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(L_ENCODER_A_PIN), handleLeftEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_ENCODER_B_PIN), handleLeftEncoderB, CHANGE);

  pinMode(R_ENCODER_A_PIN, INPUT);
  pinMode(R_ENCODER_B_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_A_PIN), handleRightEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_B_PIN), handleRightEncoderB, CHANGE);

  ledcAttachPin(L_MOTOR_PWM_PIN, L_PWM_CHANNEL);
  ledcSetup(L_PWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(R_MOTOR_PWM_PIN, R_PWM_CHANNEL);
  ledcSetup(R_PWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
}

// Function to set up IR and servo for beacon tracking
void setupBeaconTracking() {
  beaconServo.attach(SERVO_PIN);
  pinMode(PHOTOTRANSISTOR_PIN1, INPUT);
  pinMode(PHOTOTRANSISTOR_PIN2, INPUT);
  beaconServo.write(servoPos);
}

// Function to set up the Vive sensors
void setupVive() {
  frontVive.begin();
  rearVive.begin();
}
/**************************************************************/

// The main set up fucntion
void setup() {

  // Begin serial monitor
  Serial.begin(115200);

  // Call function to set up the TOF sensors
  setupTOFSensors();

  // Call function to Set up WiFi for AP mode
  setupWiFi();

  // Call function Set up the routines from HTML510
  setupHandlerRoutines();

  // Call function to set up the pins and PWM
  setupPins();

  // Call function to set up the beacon tracking pins
  setupBeaconTracking();

  // Call function to setup Vive
  setupVive();

  // Call function to setup timer -- commented out because it caused issues, not used during graded evaluation
  //setupTimer();

  lastTime = millis();
}

// The main loop function
void loop() {
    h.serve();
    delay(10);

    // Based on the current state, enter a mode to perform a task if selected
    switch(currentMode) {
        case WALL_FOLLOWING:
            performWallFollowing();
            break;
        case BEACON_TRACKING:
            runBeaconDetection();
            delay(100);
            break;
        case POLICE_TRACKING:
            runLocatePoliceCar();
            delay(1000);
            break;
        case IDLE:
        default:
            runPIDControl(false); // Default behavior
            break;
    }
}
