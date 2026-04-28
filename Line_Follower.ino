// ============================================
// LINE FOLLOWER ROBOT - TB6612FNG VERSION
// PID Controlled, Optimized for Speed & Accuracy
// ============================================

#include <Wire.h>

// ========== PIN DEFINITIONS ==========
// IR Sensor Array
#define IR0  A0
#define IR1  A1
#define IR2  A2
#define IR3  A3
#define IR4  A4
#define IR5  A5
#define IR6  A6
#define IR7  A7

// Left Motor Control (Motor A)
#define LEFT_IN1   3   // Direction pin 1
#define LEFT_IN2   2   // Direction pin 2
#define LEFT_PWM   11  // PWM speed control (D11 only!)

// Right Motor Control (Motor B)
#define RIGHT_IN1  5   // Direction pin 1
#define RIGHT_IN2  6   // Direction pin 2
#define RIGHT_PWM  9   // PWM speed control (D9 only!)

// Standby pin (optional)
#define STANDBY    12  // Can set to HIGH to enable, LOW to disable

// ========== SENSOR CONFIGURATION ==========
#define NUM_SENSORS       8
#define THRESHOLD_VALUE   500  // Adjust based on your environment
#define SENSOR_SMOOTHING  2

// ========== PID PARAMETERS ==========
float Kp = 0.18;    // Proportional gain
float Ki = 0.00;    // Integral gain
float Kd = 0.35;    // Derivative gain

// ========== MOTOR SPEED PARAMETERS ==========
#define BASE_SPEED           150   // Base motor speed (0-255)
#define MAX_SPEED            255   // Maximum motor speed
#define MIN_SPEED            100   // Minimum motor speed

// ========== VARIABLES ==========
int sensorValues[NUM_SENSORS];
float linePosition = 0;
float lastError = 0;
float integralError = 0;
float derivativeError = 0;
unsigned long lastTime = 0;

// ========== SETUP ==========
void setup() {
  Serial.begin(9600);
  
  // Motor Direction Pins as Output
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  
  // Motor PWM Pins as Output
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  
  // Standby pin (optional)
  pinMode(STANDBY, OUTPUT);
  digitalWrite(STANDBY, HIGH);  // Enable motors
  
  // IR Sensor Pins as Input
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(A0 + i, INPUT);
  }
  
  delay(1000);
  Serial.println("Line Follower Ready (TB6612FNG Version)!");
  Serial.println("Kp=" + String(Kp) + " Kd=" + String(Kd) + " Speed=" + String(BASE_SPEED));
}

// ========== MAIN LOOP ==========
void loop() {
  // Read sensor values
  readSensors();
  
  // Calculate line position (weighted average)
  calculateLinePosition();
  
  // Calculate PID error
  float error = linePosition;
  
  // Time delta for derivative calculation
  unsigned long currentTime = millis();
  float timeDelta = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  if (timeDelta > 0.1) timeDelta = 0.1;
  
  // Integral term (with anti-windup)
  integralError += error * timeDelta;
  if (integralError > 100) integralError = 100;
  if (integralError < -100) integralError = -100;
  
  // Derivative term
  if (timeDelta > 0) {
    derivativeError = (error - lastError) / timeDelta;
  }
  
  // PID Controller Output
  float pidOutput = (Kp * error) + (Ki * integralError) + (Kd * derivativeError);
  pidOutput = constrain(pidOutput, -100, 100);
  
  // Calculate motor speeds
  int leftSpeed = BASE_SPEED + pidOutput;
  int rightSpeed = BASE_SPEED - pidOutput;
  
  // Constrain speeds
  leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);
  
  // Drive motors using TB6612FNG
  driveMotors(leftSpeed, rightSpeed);
  
  // Update last error
  lastError = error;
  
  // Optional: Debug output (comment out for competition)
  // printDebugInfo(error, pidOutput, leftSpeed, rightSpeed);
  
  delay(10);  // ~100Hz control loop
}

// ========== SENSOR READING ==========
void readSensors() {
  static int lastValues[NUM_SENSORS];
  
  int rawValues[NUM_SENSORS];
  
  // Read all sensors
  rawValues[0] = analogRead(IR0);
  rawValues[1] = analogRead(IR1);
  rawValues[2] = analogRead(IR2);
  rawValues[3] = analogRead(IR3);
  rawValues[4] = analogRead(IR4);
  rawValues[5] = analogRead(IR5);
  rawValues[6] = analogRead(IR6);
  rawValues[7] = analogRead(IR7);
  
  // Exponential smoothing filter
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = (rawValues[i] * 0.4) + (lastValues[i] * 0.6);
    lastValues[i] = sensorValues[i];
    
    // Convert to binary: 1 if on line (dark), 0 if off line (white)
    sensorValues[i] = (sensorValues[i] > THRESHOLD_VALUE) ? 1 : 0;
  }
}

// ========== CALCULATE LINE POSITION ==========
// Returns -35 to +35 where 0 is center
void calculateLinePosition() {
  int weightedSum = 0;
  int sensorSum = 0;
  
  // Weights for each sensor (center = 0)
  int weights[NUM_SENSORS] = {-35, -25, -15, -5, 5, 15, 25, 35};
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    weightedSum += (sensorValues[i] * weights[i]);
    sensorSum += sensorValues[i];
  }
  
  // Calculate position
  if (sensorSum > 0) {
    linePosition = weightedSum / sensorSum;
  } else {
    // Line lost - extrapolate from last position
    linePosition = lastError * 1.5;
  }
  
  // Smooth the position
  linePosition = (linePosition * 0.6) + (lastError * 0.4);
}

// ========== MOTOR CONTROL FOR TB6612FNG ==========
void driveMotors(int leftSpeed, int rightSpeed) {
  
  // LEFT MOTOR (Motor A)
  // Set direction based on speed sign
  if (leftSpeed > 0) {
    // Forward
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
  } else if (leftSpeed < 0) {
    // Reverse
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    leftSpeed = -leftSpeed;  // Make positive for PWM
  } else {
    // Stop (both pins low)
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
  }
  analogWrite(LEFT_PWM, leftSpeed);
  
  // RIGHT MOTOR (Motor B)
  // Set direction based on speed sign
  if (rightSpeed > 0) {
    // Forward
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
  } else if (rightSpeed < 0) {
    // Reverse
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    rightSpeed = -rightSpeed;  // Make positive for PWM
  } else {
    // Stop (both pins low)
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
  }
  analogWrite(RIGHT_PWM, rightSpeed);
}

// ========== DEBUG OUTPUT ==========
void printDebugInfo(float error, float pidOutput, int leftSpeed, int rightSpeed) {
  Serial.print("Sensors: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
  }
  Serial.print("| Error: ");
  Serial.print(error);
  Serial.print("| PID: ");
  Serial.print(pidOutput);
  Serial.print("| Left: ");
  Serial.print(leftSpeed);
  Serial.print("| Right: ");
  Serial.println(rightSpeed);
}