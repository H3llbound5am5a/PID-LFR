/*
 * High-Performance PID Line Follower (Using MotorDriver.h)
 * ===============================================================
 *
 * Description:
 * This code implements a faster, more responsive PID algorithm. It's
 * designed for higher speeds and better cornering by using more
 * aggressive error correction and allowing motors to reverse for
 * sharp point-turns. It uses the MotorDriver.h library.
 *
 * Hardware Requirements:
 * 1. Arduino Uno R3 Board
 * 2. L293D Motor Driver Shield (that works with MotorDriver.h)
 * 3. 2x N20 DC Motors
 * 4. 8-Channel Analog IR Sensor Array
 * 5. External Power Supply for motors
 *
 * *** IMPORTANT WIRING ***
 * - Left Motor -> Connect to shield terminal M3
 * - Right Motor -> Connect to shield terminal M4
 * - Power Jumper on shield must be REMOVED.
 *
 * Sensor Array Connection:
 * - S1 (Leftmost) -> Arduino Digital Pin 2
 * - S8 (Rightmost) -> Arduino Digital Pin 13  // *** MOVED from Pin 4 to avoid conflict ***
 * - S2 -> A0, S3 -> A1, S4 -> A2, S5 -> A3, S6 -> A4, S7 -> A5
 *
 * Library Requirement:
 * You must have the "MotorDriver.h" library installed.
 *
 * Author: Veteran Line Follower Maker
 * Version: 3.1 (Pin conflict fix)
 */

// --- LIBRARIES ---
#include <MotorDriver.h>

// --- MOTOR SETUP ---
MotorDriver m; // Create a motor driver object

// --- PID CONTROL CONSTANTS (NEEDS RE-TUNING) ---
// These values are a starting point for the new algorithm.
// You will need to re-tune them for optimal performance.
double Kp = 0.07;
double Ki = 0.00005;
double Kd = 0.4;

// --- ROBOT PARAMETERS ---
const int MAX_SPEED = 255; // Allow full speed
const int BASE_SPEED = 180; // A higher base speed

// --- SENSOR AND ERROR CALCULATION ---
const int NUM_SENSORS = 8;
const int NUM_ANALOG_SENSORS = 6;
const int SENSOR_MAX_ERROR = 4000; // Max error value when line is lost

int analogSensorPins[NUM_ANALOG_SENSORS] = {A0, A1, A2, A3, A4, A5};
// *** MODIFIED to use Pin 13 for the rightmost sensor ***
int digitalSensorPins[] = {2, 13}; // S1 -> D2, S8 -> D13
int sensorValues[NUM_SENSORS];

int sensorThreshold = 500;

// --- PID CALCULATION VARIABLES ---
double error = 0;
double previousError = 0;
double integral = 0;
double derivative = 0;
double pidValue = 0;

// Anti-windup for the integral term
const double INTEGRAL_MAX = 5000;

void setup() {
  // Set the digital sensor pins to INPUT
  pinMode(digitalSensorPins[0], INPUT);
  pinMode(digitalSensorPins[1], INPUT);

  Serial.begin(9600);
  delay(2000);
  Serial.println("Starting High-Performance PID Follower...");
}

void loop() {
  readSensors();
  calculateError();
  calculatePID();
  controlMotors();
  printDebugInfo(); // Uncomment for tuning
}

// --- CORE FUNCTIONS ---

/**
 * @brief Reads all 8 sensors using a hybrid analog/digital approach.
 */
void readSensors() {
  // Read the leftmost digital sensor (S1)
  sensorValues[0] = (digitalRead(digitalSensorPins[0]) == LOW) ? 1023 : 0;

  // Read the 6 middle analog sensors (S2-S7)
  for (int i = 0; i < NUM_ANALOG_SENSORS; i++) {
    sensorValues[i + 1] = analogRead(analogSensorPins[i]);
  }

  // Read the rightmost digital sensor (S8)
  sensorValues[7] = (digitalRead(digitalSensorPins[1]) == LOW) ? 1023 : 0;
}

/**
 * @brief Calculates a weighted average for all 8 sensors.
 * This version uses a more aggressive error value when the line is lost.
 */
void calculateError() {
  long weightedSum = 0;
  int numActiveSensors = 0;
  bool onLine = false;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] > sensorThreshold) {
      onLine = true;
      numActiveSensors++;
      // Weights for 8 sensors: -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
      weightedSum += (long)(i - (NUM_SENSORS - 1) / 2.0) * 1000;
    }
  }

  if (!onLine) {
    // If the line is lost, make a hard turn based on the last known position.
    if (previousError > 0) { // Was on the right side of the line
      error = SENSOR_MAX_ERROR;
    } else { // Was on the left side of the line
      error = -SENSOR_MAX_ERROR;
    }
  } else {
    error = (double)weightedSum / numActiveSensors;
  }
}

/**
 * @brief Computes the PID output value with integral windup protection.
 */
void calculatePID() {
  integral += error;

  // Prevent integral windup: clamp the integral term to a max/min value.
  integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  
  derivative = error - previousError;
  pidValue = (Kp * error) + (Ki * integral) + (Kd * derivative);
  previousError = error;
}

/**
 * @brief Sets the speed of the motors. This version allows one motor to go
 * in reverse for extremely sharp, agile turns.
 */
void controlMotors() {
  double leftMotorSpeed = BASE_SPEED - pidValue;
  double rightMotorSpeed = BASE_SPEED + pidValue;

  // --- Left Motor Control (M3) ---
  if (leftMotorSpeed >= 0) {
    // Move forward
    m.motor(3, FORWARD, constrain(leftMotorSpeed, 0, MAX_SPEED));
  } else {
    // Move backward for sharp turns
    m.motor(3, BACKWARD, constrain(abs(leftMotorSpeed), 0, MAX_SPEED));
  }

  // --- Right Motor Control (M4) ---
  if (rightMotorSpeed >= 0) {
    // Move forward
    m.motor(4, FORWARD, constrain(rightMotorSpeed, 0, MAX_SPEED));
  } else {
    // Move backward for sharp turns
    m.motor(4, BACKWARD, constrain(abs(rightMotorSpeed), 0, MAX_SPEED));
  }
}

// --- HELPER AND DEBUGGING FUNCTIONS ---

/**
 * @brief Prints PID variables to the serial monitor for debugging.
 */
void printDebugInfo() {
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print("\tPID: ");
  Serial.print(pidValue);
  Serial.print("\tL_Spd: ");
  Serial.print(BASE_SPEED - pidValue);
  Serial.print("\tR_Spd: ");
  Serial.println(BASE_SPEED + pidValue);
}
