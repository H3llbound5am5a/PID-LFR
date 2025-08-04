/*
 * High-Performance PID Line Followe
 * ===============================================================
 *
 * Description:
 * This version features a "one-spin" autonomous calibration that runs
 * automatically every time the robot is powered on. It does not save
 * the calibration value to permanent memory. This version is tuned
 * for better performance on sharp 90-degree turns.
 *
 * Hardware Requirements:
 * 1. Arduino Uno R3 Board
 * 2. L293D Motor Driver Shield
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
 * - S8 (Rightmost) -> Arduino Digital Pin 13
 * - S2 -> A0, S3 -> A1, S4 -> A2, S5 -> A3, S6 -> A4, S7 -> A5
 *
 * Library Requirement:
 * You must have the "MotorDriver.h" library installed.
 *
 * Author: ded
 * Version: 4.1 (90-Degree Turn Enhancement)
 */

// --- LIBRARIES ---
#include <MotorDriver.h>

// --- MOTOR SETUP ---
MotorDriver m; // Create a motor driver object

// --- PID CONTROL CONSTANTS (NEEDS RE-TUNING) ---
// *** MODIFIED: More aggressive values for sharper turns. ***
double Kp = 0.09;     // Increased Kp for a stronger, faster reaction to error.
double Ki = 0.00005;  // Ki remains small to prevent overshoot on straights.
double Kd = 0.5;      // Increased Kd to dampen the stronger Kp and reduce oscillation.

// --- ROBOT PARAMETERS ---
const int MAX_SPEED = 255;
// *** NOTE: If still failing turns, try reducing BASE_SPEED to 150 or 160. ***
const int BASE_SPEED = 180;
const int CALIBRATION_SPEED = 150; // Speed for rotation/movement during calibration

// --- SENSOR AND ERROR CALCULATION ---
const int NUM_SENSORS = 8;
const int NUM_ANALOG_SENSORS = 6;
// *** MODIFIED: Increased max error for a more decisive turn when the line is lost. ***
const int SENSOR_MAX_ERROR = 5000;

int analogSensorPins[NUM_ANALOG_SENSORS] = {A0, A1, A2, A3, A4, A5};
int digitalSensorPins[] = {2, 13}; // S1 -> D2, S8 -> D13
int sensorValues[NUM_SENSORS];

// This value is set by the mandatory calibration routine on startup.
int sensorThreshold = 500;

// --- PID CALCULATION VARIABLES ---
double error = 0;
double previousError = 0;
double integral = 0;
double derivative = 0;
double pidValue = 0;
const double INTEGRAL_MAX = 5000;

void setup() {
  pinMode(digitalSensorPins[0], INPUT);
  pinMode(digitalSensorPins[1], INPUT);
  Serial.begin(9600);
  delay(1000);

  // Always run the autonomous calibration routine on every startup.
  calibrateSensors();

  Serial.println("Starting High-Performance PID Follower...");
  Serial.print("Using newly calibrated Sensor Threshold: ");
  Serial.println(sensorThreshold);
  delay(2000); // Give user time to let go of the robot.
}

void loop() {
  readSensors();
  calculateError();
  calculatePID();
  controlMotors();
  // printDebugInfo(); // Uncomment for tuning PID constants
}

// --- AUTONOMOUS CALIBRATION FUNCTION ---
/**
 * @brief An autonomous routine that runs on every startup. The robot
 * rotates 360 degrees to find the min (black) and max (white)
 * sensor values to set the threshold.
 */
void calibrateSensors() {
  Serial.println("--- One-Spin Autonomous Calibration ---");
  Serial.println("Place robot ON the BLACK line now.");
  Serial.println("Calibration will start in 5 seconds...");
  delay(5000);

  // --- Calibrate by rotating and finding min/max values ---
  Serial.println("Scanning... Rotating 360 degrees.");
  
  int minReading = 1023; // Will hold the low value (black)
  int maxReading = 0;    // Will hold the high value (white)

  m.motor(3, FORWARD, CALIBRATION_SPEED);
  m.motor(4, BACKWARD, CALIBRATION_SPEED);

  // Rotate for a set duration to complete a ~360 degree turn.
  // Adjust the loop count (150) if the robot over or under-rotates.
  for(int i = 0; i < 150; i++){
    for (int j = 0; j < NUM_ANALOG_SENSORS; j++) {
      int currentReading = analogRead(analogSensorPins[j]);
      if (currentReading < minReading) {
        minReading = currentReading;
      }
      if (currentReading > maxReading) {
        maxReading = currentReading;
      }
    }
    delay(10);
  }

  // Stop the motors
  m.motor(3, BRAKE, 0);
  m.motor(4, BRAKE, 0);

  // Calculate and assign the new threshold directly to the global variable
  sensorThreshold = (minReading + maxReading) / 2;
  
  Serial.println("\n--- CALIBRATION COMPLETE ---");
  Serial.print("Min (Black) Reading: ");
  Serial.println(minReading);
  Serial.print("Max (White) Reading: ");
  Serial.println(maxReading);
  Serial.print("New threshold calculated and set: ");
  Serial.println(sensorThreshold);
  delay(1000);
}


// --- CORE FUNCTIONS ---

void readSensors() {
  // Logic for inverted sensors (white is high, black is low)
  sensorValues[0] = (digitalRead(digitalSensorPins[0]) == HIGH) ? 0 : 1023;
  for (int i = 0; i < NUM_ANALOG_SENSORS; i++) {
    sensorValues[i + 1] = analogRead(analogSensorPins[i]);
  }
  sensorValues[7] = (digitalRead(digitalSensorPins[1]) == HIGH) ? 0 : 1023;
}

void calculateError() {
  long weightedSum = 0;
  int numActiveSensors = 0;
  bool onLine = false;

  for (int i = 0; i < NUM_SENSORS; i++) {
    // Logic for inverted sensors (lower reading means on the black line)
    if (sensorValues[i] < sensorThreshold) {
      onLine = true;
      numActiveSensors++;
      weightedSum += (long)(i - (NUM_SENSORS - 1) / 2.0) * 1000;
    }
  }

  if (!onLine) {
    error = (previousError > 0) ? SENSOR_MAX_ERROR : -SENSOR_MAX_ERROR;
  } else {
    error = (double)weightedSum / numActiveSensors;
  }
}

void calculatePID() {
  integral += error;
  integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  derivative = error - previousError;
  pidValue = (Kp * error) + (Ki * integral) + (Kd * derivative);
  previousError = error;
}

void controlMotors() {
  double leftMotorSpeed = BASE_SPEED - pidValue;
  double rightMotorSpeed = BASE_SPEED + pidValue;

  if (leftMotorSpeed >= 0) {
    m.motor(3, FORWARD, constrain(leftMotorSpeed, 0, MAX_SPEED));
  } else {
    m.motor(3, BACKWARD, constrain(abs(leftMotorSpeed), 0, MAX_SPEED));
  }

  if (rightMotorSpeed >= 0) {
    m.motor(4, FORWARD, constrain(rightMotorSpeed, 0, MAX_SPEED));
  } else {
    m.motor(4, BACKWARD, constrain(abs(rightMotorSpeed), 0, MAX_SPEED));
  }
}

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

