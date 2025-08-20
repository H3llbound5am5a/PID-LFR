// ============================================================================
// High-Performance PID Line Follower (Competition Build v16.2)
//
// Implements a PID controller with a state machine for handling sharp turns,
// intersections, and line recovery. Aggressively tuned for speed.
//
// Hardware: Arduino Mega, L293D Motor Shield, 11 IR sensors, N20 motors.
// ============================================================================

#include <MotorDriver.h>
#include <EEPROM.h>

MotorDriver m;

// --- System Configuration & Tuning ---
const bool DEBUG_MODE = true;

// Motor direction correction
const bool INVERT_LEFT_MOTOR = false;
const bool INVERT_RIGHT_MOTOR = false;

// --- Robot Constants ---
const int SWITCH_PIN = 22;

const int MAX_SPEED = 230;
const int TURN_SPEED = 200;
const int MIN_SPEED = 90;
const int RECOVERY_SPEED = 120; // Speed for the backup-and-turn recovery maneuver

const int NUM_SENSORS = 11;
const int sensorPins[NUM_SENSORS] = {A13, A0, A1, A2, A3, A14, A4, A8, A9, A12, A15};
const int LOOKAHEAD_INDEX = 5; // The center sensor, used for intersection logic
const int INTERSECTION_THRESHOLD = 5; // Min sensors seeing the line to be considered an intersection

enum RobotState { FOLLOWING, SHARP_TURN, CROSSING_INTERSECTION, RECOVERY };

// Initial PID gains for tuning
double Kp = 3.5, Ki = 0.0005, Kd = 9.0;
const int SETPOINT = 4000; // The target value for a centered line position

const unsigned long LINE_LOST_TIMEOUT = 300; // ms before triggering recovery state

// --- Global Variables ---
unsigned int sensorMin[NUM_SENSORS];
unsigned int sensorMax[NUM_SENSORS];
double error = 0, lastError = 0, integral = 0, derivative = 0;
bool isRunning = false;
RobotState currentState = FOLLOWING;
unsigned long stateTimer = 0; // Generic timer for state-based delays
unsigned long lastLineSeen = 0;
unsigned long intersectionCooldownTimer = 0;
const int INTERSECTION_COOLDOWN = 250; // Prevent re-triggering intersection logic immediately

// --- Function Prototypes ---
void handleSwitch();
void calibrateSensors();
void stopMotors();
void runMotor(int motor, int speed);
int readLine(bool &lineFound, bool &intersectionDetected, bool &leftWing, bool &rightWing);

void setup() {
    Serial.begin(115200);

    // Sanity check for power-related resets (brownouts)
    Serial.println("\n\n===========================");
    Serial.println("--- ROBOT BOOT / RESET ---");
    Serial.println("===========================");

    for (int i = 0; i < NUM_SENSORS; i++) pinMode(sensorPins[i], INPUT);
    pinMode(SWITCH_PIN, INPUT_PULLUP);

    // Check for a magic byte to see if calibration data exists in EEPROM
    if (EEPROM.read(0) == 123) {
        EEPROM.get(1, sensorMin);
        EEPROM.get(1 + sizeof(sensorMin), sensorMax);
        Serial.println("Calibration loaded from EEPROM.");
    } else {
        // Initialize with default values if no calibration is found
        for (int i = 0; i < NUM_SENSORS; i++) {
            sensorMin[i] = 1023;
            sensorMax[i] = 0;
        }
        Serial.println("No calibration found. Please calibrate.");
    }

    Serial.println("--- Line Follower Ready ---");
    Serial.println("Short press = Start/Stop");
    Serial.println("Long press (2s) = Calibrate");
    if (DEBUG_MODE) {
        Serial.println("*** DEBUG MODE IS ON ***");
    }
}

void loop() {
    handleSwitch();

    if (isRunning) {
        bool lineFound = false, intersectionDetected = false, leftWing = false, rightWing = false;
        int position = readLine(lineFound, intersectionDetected, leftWing, rightWing);

        // Pre-calculate derivative to detect sharp turns/jerks early.
        // This is a quick way to check for a massive, sudden change in line position.
        derivative = (position - SETPOINT) - lastError;
        if ((abs(derivative) > 1800 || leftWing || rightWing) && lineFound) {
            currentState = SHARP_TURN;
        }

        // Main state machine
        switch (currentState) {
            case FOLLOWING:
                if (lineFound) {
                    lastLineSeen = millis();

                    // Standard PID calculation
                    error = position - SETPOINT;
                    integral += error;
                    integral = constrain(integral, -20000, 20000); // Prevent integral windup
                    derivative = error - lastError;
                    lastError = error;

                    double pidCorrection = Kp * error + Ki * integral + Kd * derivative;

                    // Dynamic speed control: slow down proportionally to the error (i.e., in curves)
                    int speedReduction = map(abs(error), 0, SETPOINT, 0, MAX_SPEED - MIN_SPEED);
                    int currentSpeed = MAX_SPEED - speedReduction;
                    currentSpeed = constrain(currentSpeed, MIN_SPEED, MAX_SPEED);

                    // Apply correction to the motors
                    int leftSpeed = currentSpeed + pidCorrection;
                    int rightSpeed = currentSpeed - pidCorrection;

                    runMotor(3, leftSpeed);
                    runMotor(4, rightSpeed);

                    // Trigger intersection state if detected and not on cooldown.
                    // The lookahead sensor check helps confirm it's a true intersection, not a glitch.
                    if (intersectionDetected && millis() - intersectionCooldownTimer > INTERSECTION_COOLDOWN &&
                        analogRead(sensorPins[LOOKAHEAD_INDEX]) < 600) {
                        currentState = CROSSING_INTERSECTION;
                        stateTimer = millis();
                    }
                } else { // Line is not visible
                    // If we've been off the line for too long, enter full recovery mode
                    if (millis() - lastLineSeen > LINE_LOST_TIMEOUT) {
                        currentState = RECOVERY;
                    } else {
                        // Otherwise, do a quick pivot turn in the last known direction of the line
                        if (lastError < 0) { // Line was to the left
                            runMotor(3, -TURN_SPEED);
                            runMotor(4, TURN_SPEED);
                        } else { // Line was to the right
                            runMotor(3, TURN_SPEED);
                            runMotor(4, -TURN_SPEED);
                        }
                    }
                }
                break;

            case SHARP_TURN:
                // Once we find the line again, go back to normal following
                if (lineFound) {
                    currentState = FOLLOWING;
                    integral = 0; // Reset PID terms to prevent a massive jerk
                    lastError = 0;
                } else {
                    // Turn aggressively based on which wing sensor was triggered,
                    // or the last known error if no wing sensor is active.
                    if (leftWing) {
                        runMotor(3, -TURN_SPEED);
                        runMotor(4, TURN_SPEED);
                    } else if (rightWing) {
                        runMotor(3, TURN_SPEED);
                        runMotor(4, -TURN_SPEED);
                    } else {
                        if (lastError < 0) {
                            runMotor(3, -TURN_SPEED);
                            runMotor(4, TURN_SPEED);
                        } else {
                            runMotor(3, TURN_SPEED);
                            runMotor(4, -TURN_SPEED);
                        }
                    }
                }
                break;

            case CROSSING_INTERSECTION:
                // Drive straight for a fixed duration to clear the intersection
                if (millis() - stateTimer > 120) {
                    currentState = FOLLOWING;
                    intersectionCooldownTimer = millis();
                    integral = 0;
                    lastError = 0;
                } else {
                    runMotor(3, 180);
                    runMotor(4, 180);
                }
                break;

            case RECOVERY:
                // A more drastic maneuver for when the line is truly lost
                runMotor(3, -RECOVERY_SPEED); // 1. Back up briefly
                runMotor(4, -RECOVERY_SPEED);
                delay(150);

                if (lastError < 0) { // 2. Pivot in the last known direction
                    runMotor(3, -TURN_SPEED);
                    runMotor(4, TURN_SPEED);
                } else {
                    runMotor(3, TURN_SPEED);
                    runMotor(4, -TURN_SPEED);
                }
                delay(200); // Hold the turn for a bit
                currentState = FOLLOWING; // 3. Go back to hunting for the line
                break;
        }
    } else {
        stopMotors();
    }
}

// Handles short press (start/stop) and long press (calibrate) of the main button.
void handleSwitch() {
    static int lastButtonState = HIGH;
    static unsigned long pressStart = 0;
    static bool longPressHandled = false;

    int state = digitalRead(SWITCH_PIN);

    // Rising edge: button was just pressed
    if (state == LOW && lastButtonState == HIGH) {
        pressStart = millis();
        longPressHandled = false;
    }

    // Falling edge: button was just released
    if (state == HIGH && lastButtonState == LOW) {
        // If it wasn't a long press, treat it as a short press
        if (!longPressHandled && millis() - pressStart < 1500) {
            isRunning = !isRunning;
            if (isRunning) {
                Serial.println("BOT STARTED");
                currentState = FOLLOWING;
                integral = 0; lastError = 0; // Reset PID on start
            } else {
                Serial.println("BOT STOPPED");
            }
        }
    }

    // Check for a long press while the button is held down
    if (state == LOW && !longPressHandled) {
        if (millis() - pressStart >= 2000) {
            isRunning = false; // Always stop the bot for calibration
            stopMotors();
            calibrateSensors();
            longPressHandled = true; // Flag to prevent short press from firing on release
        }
    }

    lastButtonState = state;
}

// Rotates the robot over the line to find min/max sensor readings.
void calibrateSensors() {
    Serial.println("Starting calibration... Turn the bot over the line.");
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorMin[i] = 1023;
        sensorMax[i] = 0;
    }

    unsigned long start = millis();
    while (millis() - start < 2500) {
        runMotor(3, 135);
        runMotor(4, -135);
        for (int j = 0; j < NUM_SENSORS; j++) {
            int val = analogRead(sensorPins[j]);
            if (val < sensorMin[j]) sensorMin[j] = val;
            if (val > sensorMax[j]) sensorMax[j] = val;
        }
    }
    stopMotors();

    // Persist calibration data to non-volatile memory
    EEPROM.put(1, sensorMin);
    EEPROM.put(1 + sizeof(sensorMin), sensorMax);
    EEPROM.write(0, 123); // Magic byte
    Serial.println("Calibration complete and saved!");
}

// Reads all sensors and calculates a weighted average for the line position.
int readLine(bool &lineFound, bool &intersectionDetected, bool &leftWing, bool &rightWing) {
    unsigned long weightedSum = 0;
    unsigned int sum = 0;
    int calibrated[NUM_SENSORS];
    int activeCount = 0;
    bool onLine = false;

    for (int i = 0; i < NUM_SENSORS; i++) {
        int raw = analogRead(sensorPins[i]);
        // Avoid division by zero if a sensor wasn't calibrated properly
        if (sensorMax[i] <= sensorMin[i]) {
            calibrated[i] = 0;
        } else {
            // Map the raw sensor value to a 0-1000 range, where 1000 is directly over the line
            calibrated[i] = map(raw, sensorMin[i], sensorMax[i], 1000, 0);
        }
        calibrated[i] = constrain(calibrated[i], 0, 1000);

        // A sensor is "active" if it's reading the line with some confidence
        if (calibrated[i] > 200) {
            activeCount++;
            // Don't count the outer wing sensors for the main 'onLine' flag
            if (i > 0 && i < NUM_SENSORS - 1) {
                onLine = true;
            }
        }
    }

    // Calculate the weighted average only using the main sensor array (ignore wings)
    for (int i = 1; i < NUM_SENSORS - 1; i++) {
        weightedSum += (unsigned long)calibrated[i] * (i - 1) * 1000;
        sum += calibrated[i];
    }

    lineFound = onLine;
    intersectionDetected = (activeCount >= INTERSECTION_THRESHOLD);
    leftWing = calibrated[0] > 500;
    rightWing = calibrated[NUM_SENSORS - 1] > 500;

    // If all sensors are off the line, return an extreme value
    // based on the last known direction to encourage turning that way.
    if (sum == 0) {
        lineFound = false;
        return (lastError < 0) ? 0 : 8000;
    }

    return weightedSum / sum;
}

// Wrapper for the motor driver to handle inversion and speed constraints.
void runMotor(int motorNum, int speed) {
    bool isLeftMotor = (motorNum == 3);
    bool invert = (isLeftMotor) ? INVERT_LEFT_MOTOR : INVERT_RIGHT_MOTOR;

    if (invert) {
        speed = -speed;
    }

    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

    if (speed >= 0) {
        m.motor(motorNum, FORWARD, speed);
    } else {
        m.motor(motorNum, BACKWARD, abs(speed));
    }
}

// Brakes both motors to a hard stop.
void stopMotors() {
    m.motor(3, BRAKE, 0);
    m.motor(4, BRAKE, 0);
}
