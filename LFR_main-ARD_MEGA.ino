// ============================================================================
// High-Performance PID Line Follower (Competition Build v15.3 - L293D)
//
// This is a high-speed line follower designed for competitive tracks. It uses
// a PID algorithm to stay locked on the line, with special logic to handle
// sharp turns and intersections. It can even calibrate its own sensors and
// attempt to auto-tune its PID values.
//
// Hardware: Arduino Mega, L293D Motor Shield, 11 IR sensors, N20 motors.
// ============================================================================

#include <MotorDriver.h>
#include <EEPROM.h>

MotorDriver m;

// --- Constants ---
const int SWITCH_PIN = 22; // The pin for our start/stop/calibrate button.

// Motor speeds. We define these upfront to make tweaking easier.
const int MAX_SPEED = 230;      // The fastest the robot will go on a straight.
const int BASE_SPEED = 180;     // A good cruising speed for general line following.
const int TURN_SPEED = 200;     // Speed for making sharp, deliberate turns.
const int MIN_SPEED = 90;       // The slowest the robot will go, even on sharp curves.
const int RECOVERY_SPEED = 120; // A slow, careful speed for the recovery routine.

// Sensor setup.
const int NUM_SENSORS = 11;
// This array holds the analog pins for all our sensors.
// The first and last are the "wing" sensors for detecting sharp turns.
// The middle 9 form the main array for calculating our position on the line.
const int sensorPins[NUM_SENSORS] = {A13, A0, A1, A2, A3, A14, A4, A8, A9, A12, A15};
const int LOOKAHEAD_INDEX = 5; // The sensor that sits a bit ahead to spot intersections early.
const int INTERSECTION_THRESHOLD = 5; // How many sensors need to be on the line to call it an intersection.

// These are the different "moods" or states our robot can be in.
enum RobotState { FOLLOWING, SHARP_TURN, CROSSING_INTERSECTION, RECOVERY };

// Our starting PID values. These can be auto-tuned or loaded from memory.
double Kp = 1.9, Ki = 0.0, Kd = 1.5;
// The "perfect" sensor reading. We're always trying to get our position to this value.
// Since our main array has 9 sensors (0-8000), the middle is 4000.
const int SETPOINT = 4000;

// How long we can lose the line before we panic and enter recovery mode.
const unsigned long LINE_LOST_TIMEOUT = 300; // in milliseconds

// --- Global Variables ---
unsigned int sensorMin[NUM_SENSORS]; // Stores the lowest reading for each sensor during calibration.
unsigned int sensorMax[NUM_SENSORS]; // Stores the highest reading for each sensor.
double error = 0, lastError = 0, integral = 0, derivative = 0; // The core PID variables.
bool isRunning = false; // Is the robot currently supposed to be moving?
int currentSpeed = BASE_SPEED; // The robot's current target speed.
RobotState currentState = FOLLOWING; // The robot's current state.
unsigned long stateTimer = 0; // A general-purpose timer for states.
unsigned long lastLineSeen = 0; // When was the last time we confidently saw the line?
unsigned long intersectionCooldownTimer = 0; // Prevents seeing the same intersection multiple times.
const int INTERSECTION_COOLDOWN = 250;

// Variables for handling multiple button presses.
int pressCount = 0;
unsigned long lastPressTime = 0;
const int MULTI_PRESS_TIMEOUT = 350;

// --- Function Prototypes ---
// We list our functions here so the compiler knows about them ahead of time.
void handleSwitch();
void calibrateSensors();
void stopMotors();
int readLine(bool &lineFound, bool &intersectionDetected, bool &leftWing, bool &rightWing);
void autoTunePID();
void loadPID();

// The setup function runs once when the Arduino first powers on.
void setup() {
    Serial.begin(115200);
    for (int i = 0; i < NUM_SENSORS; i++) pinMode(sensorPins[i], INPUT);
    pinMode(SWITCH_PIN, INPUT_PULLUP);

    // Try to load a previous calibration from the Arduino's permanent memory.
    if (EEPROM.read(0) == 123) { // 123 is our magic number to check if data is valid.
        EEPROM.get(1, sensorMin);
        EEPROM.get(1 + sizeof(sensorMin), sensorMax);
        Serial.println("Calibration loaded from EEPROM.");
    } else {
        // If no calibration is found, we'll start with fresh, empty values.
        for (int i = 0; i < NUM_SENSORS; i++) {
            sensorMin[i] = 1023; // Start mins high
            sensorMax[i] = 0;    // Start maxs low
        }
        Serial.println("No calibration found. Please calibrate.");
    }

    loadPID(); // Load any saved PID values.

    Serial.println("--- Line Follower Ready ---");
    Serial.println("Short press = Start/Stop");
    Serial.println("Long press (2s) = Calibrate");
    Serial.println("5 quick presses = Auto-Tune PID");
}

// The loop function runs over and over again, forever. This is the robot's "brain".
void loop() {
    handleSwitch(); // First, always check if the user is pressing the button.

    if (isRunning) {
        bool lineFound = false, intersectionDetected = false, leftWing = false, rightWing = false;
        int position = readLine(lineFound, intersectionDetected, leftWing, rightWing);

        // Check if we're heading into a sharp turn. A big, sudden change in position is a good clue.
        derivative = (position - SETPOINT) - lastError;
        if ((abs(derivative) > 1800 || leftWing || rightWing) && lineFound) {
            currentState = SHARP_TURN;
        }

        // This is a state machine. The robot's behavior depends on its current state.
        switch (currentState) {
            // This is the normal state: happily following the line.
            case FOLLOWING:
                if (lineFound) {
                    lastLineSeen = millis(); // We see the line, so we reset our "lost" timer.

                    // The core PID calculation.
                    error = position - SETPOINT; // How far are we from the center?
                    integral += error; // Accumulate error over time to fix steady-state offsets.
                    integral = constrain(integral, -20000, 20000); // Don't let the integral get out of control.
                    derivative = error - lastError; // How fast is the error changing? This helps us anticipate.
                    lastError = error; // Remember the error for the next loop.

                    // Let's adjust our PID gains based on speed. This can improve stability.
                    double speedFactor = (double)currentSpeed / MAX_SPEED;
                    double pidCorrection = (Kp * speedFactor) * error + (Ki * integral) + (Kd * speedFactor) * derivative;

                    // The further we are from the line, the more we should slow down.
                    int speedReduction = map(abs(error), 0, SETPOINT, 0, MAX_SPEED - MIN_SPEED);
                    currentSpeed = MAX_SPEED - speedReduction;
                    currentSpeed = constrain(currentSpeed, MIN_SPEED, MAX_SPEED);

                    // Calculate the final speed for each motor and send the commands.
                    double leftSpeed = currentSpeed + pidCorrection;
                    double rightSpeed = currentSpeed - pidCorrection;
                    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
                    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

                    m.motor(3, (leftSpeed >= 0) ? FORWARD : BACKWARD, abs(leftSpeed));
                    m.motor(4, (rightSpeed >= 0) ? FORWARD : BACKWARD, abs(rightSpeed));

                    // Check if our look-ahead sensor has spotted an intersection.
                    if (intersectionDetected && millis() - intersectionCooldownTimer > INTERSECTION_COOLDOWN &&
                        analogRead(sensorPins[LOOKAHEAD_INDEX]) < 600) {
                        currentState = CROSSING_INTERSECTION;
                        stateTimer = millis();
                    }
                } else { // If we don't see the line...
                    // Have we been lost for too long?
                    if (millis() - lastLineSeen > LINE_LOST_TIMEOUT) {
                        currentState = RECOVERY; // Time to panic.
                    } else {
                        // If we just barely lost the line, let's try to find it again
                        // by making a sharp turn in the direction we last saw it.
                        if (lastError < 0) { // If the line was to our left...
                            m.motor(3, BACKWARD, TURN_SPEED); // Pivot left.
                            m.motor(4, FORWARD, TURN_SPEED);
                        } else { // If the line was to our right...
                            m.motor(3, FORWARD, TURN_SPEED);  // Pivot right.
                            m.motor(4, BACKWARD, TURN_SPEED);
                        }
                    }
                }
                break;

            // This state is for handling very sharp, pre-defined turns.
            case SHARP_TURN:
                if (lineFound) {
                    // We found the line again! Let's go back to following it.
                    currentState = FOLLOWING;
                    integral = 0; // Reset PID terms to avoid a sudden jerk.
                    lastError = 0;
                } else {
                    // Keep turning until we find the line.
                    if (leftWing) {
                        m.motor(3, BACKWARD, TURN_SPEED);
                        m.motor(4, FORWARD, TURN_SPEED);
                    } else if (rightWing) {
                        m.motor(3, FORWARD, TURN_SPEED);
                        m.motor(4, BACKWARD, TURN_SPEED);
                    } else {
                        // If a wing sensor didn't trigger this, turn based on our last known position.
                        if (lastError < 0) {
                            m.motor(3, BACKWARD, TURN_SPEED);
                            m.motor(4, FORWARD, TURN_SPEED);
                        } else {
                            m.motor(3, FORWARD, TURN_SPEED);
                            m.motor(4, BACKWARD, TURN_SPEED);
                        }
                    }
                }
                break;

            // This state helps us drive straight through an intersection without getting distracted.
            case CROSSING_INTERSECTION:
                if (millis() - stateTimer > 120) { // Drive straight for a brief moment.
                    currentState = FOLLOWING;
                    intersectionCooldownTimer = millis(); // Start cooldown to ignore the same intersection.
                    integral = 0;
                    lastError = 0;
                } else {
                    m.motor(3, FORWARD, BASE_SPEED);
                    m.motor(4, FORWARD, BASE_SPEED);
                }
                break;

            // This is our last-ditch effort to find the line if we get totally lost.
            case RECOVERY:
                m.motor(3, BACKWARD, RECOVERY_SPEED); // Back up a little bit.
                m.motor(4, BACKWARD, RECOVERY_SPEED);
                delay(150);
                
                // Turn towards where we think the line might be.
                if (lastError < 0) {
                    m.motor(3, BACKWARD, TURN_SPEED);
                    m.motor(4, FORWARD, TURN_SPEED);
                } else {
                    m.motor(3, FORWARD, TURN_SPEED);
                    m.motor(4, BACKWARD, TURN_SPEED);
                }
                delay(200); // Hold the turn for a moment.
                currentState = FOLLOWING; // Now, let's try following again.
                break;
        }
    } else {
        // If the robot isn't running, make sure the motors are stopped.
        stopMotors();
    }
}

// This function handles all the logic for the user button.
void handleSwitch() {
    static int lastButtonState = HIGH;
    static unsigned long pressStart = 0;
    static bool longPressHandled = false;

    int state = digitalRead(SWITCH_PIN);

    // Check if the button was just pressed.
    if (state == LOW && lastButtonState == HIGH) {
        pressStart = millis();
        longPressHandled = false;

        // This part counts how many times the button is pressed quickly.
        if (millis() - lastPressTime > MULTI_PRESS_TIMEOUT) {
            pressCount = 1; // If it's been a while, this is the first press.
        } else {
            pressCount++; // Otherwise, increment the count.
        }
        lastPressTime = millis();
    }

    // Check if the button was just released.
    if (state == HIGH && lastButtonState == LOW) {
        if (!longPressHandled && millis() - pressStart < 1500) { // A short press.
            if (pressCount == 1) {
                isRunning = !isRunning; // Toggle the robot's running state.
                if (isRunning) {
                    Serial.println("BOT STARTED");
                    currentState = FOLLOWING;
                    integral = 0; lastError = 0; // Reset everything for a clean start.
                    currentSpeed = BASE_SPEED;
                } else {
                    Serial.println("BOT STOPPED");
                }
            }
        }
    }

    // Check if the button is being held down for a long press.
    if (state == LOW && !longPressHandled) {
        if (millis() - pressStart >= 2000) {
            isRunning = false;
            stopMotors();
            calibrateSensors(); // A long press triggers calibration.
            longPressHandled = true;
            pressCount = 0;
        }
    }

    // Check if the button was pressed 5 times quickly.
    if (pressCount >= 5 && millis() - lastPressTime > MULTI_PRESS_TIMEOUT) {
        autoTunePID(); // 5 presses triggers the auto-tuner.
        pressCount = 0;
    }

    lastButtonState = state;
}

// This function manages the sensor calibration routine.
void calibrateSensors() {
    Serial.println("Starting calibration... Turn the bot over the line.");
    // Reset our min and max values.
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorMin[i] = 1023;
        sensorMax[i] = 0;
    }

    // Spin the robot for a few seconds to read the line and the background.
    unsigned long start = millis();
    while (millis() - start < 2500) {
        m.motor(3, FORWARD, 135);
        m.motor(4, BACKWARD, 135);
        for (int j = 0; j < NUM_SENSORS; j++) {
            int val = analogRead(sensorPins[j]);
            // As we spin, we update our min and max values for each sensor.
            if (val < sensorMin[j]) sensorMin[j] = val;
            if (val > sensorMax[j]) sensorMax[j] = val;
        }
    }
    stopMotors();

    // Save the new calibration values to permanent memory.
    EEPROM.put(1, sensorMin);
    EEPROM.put(1 + sizeof(sensorMin), sensorMax);
    EEPROM.write(0, 123); // Write our magic number so we know the data is good.
    Serial.println("Calibration complete and saved!");
}

// This is the heart of the line follower. It reads all the sensors and
// calculates a single number representing the line's position.
int readLine(bool &lineFound, bool &intersectionDetected, bool &leftWing, bool &rightWing) {
    unsigned long weightedSum = 0;
    unsigned int sum = 0;
    int calibrated[NUM_SENSORS];
    int activeCount = 0;
    bool onLine = false;

    // First, read every sensor and scale its value from 0 to 1000.
    for (int i = 0; i < NUM_SENSORS; i++) {
        int raw = analogRead(sensorPins[i]);
        // The map function rescales the raw value based on our calibration.
        if (sensorMax[i] <= sensorMin[i]) {
            calibrated[i] = 0;
        } else {
            calibrated[i] = map(raw, sensorMin[i], sensorMax[i], 1000, 0);
        }
        calibrated[i] = constrain(calibrated[i], 0, 1000);

        // If a sensor's value is high, it's probably over the line.
        if (calibrated[i] > 200) {
            activeCount++;
            if (i > 0 && i < NUM_SENSORS - 1) { // We only care about the main array here.
                onLine = true;
            }
        }
    }

    // Now, calculate the weighted average of the main array sensors.
    // This gives us a precise position instead of just "left" or "right".
    for (int i = 1; i < NUM_SENSORS - 1; i++) {
        weightedSum += (unsigned long)calibrated[i] * (i - 1) * 1000;
        sum += calibrated[i];
    }

    // Update our status flags.
    lineFound = onLine;
    intersectionDetected = (activeCount >= INTERSECTION_THRESHOLD);
    leftWing = calibrated[0] > 500;
    rightWing = calibrated[NUM_SENSORS - 1] > 500;

    // If none of the sensors see the line...
    if (sum == 0) {
        lineFound = false;
        // ...return a value that's far to the left or right, depending on where we were last.
        return (lastError < 0) ? 0 : 8000;
    }

    // Return the final calculated position.
    return weightedSum / sum;
}


// This function attempts to automatically find good PID values.
// It's based on the Ziegler-Nichols method.
void autoTunePID() {
    Serial.println("\n--- PID AUTO-TUNE STARTED ---");
    stopMotors();
    delay(1000);

    double testKp = 0.5;
    double Ku = 0; // The ultimate gain, where the robot starts oscillating.
    double Pu = 0; // The period of that oscillation.
    int oscillations = 0;
    bool goingUp = false;

    // We'll slowly increase Kp until the robot starts to oscillate consistently.
    while (testKp < 5.0) {
        Kp = testKp; Kd = 0; Ki = 0;

        unsigned long start = millis();
        lastError = 0;
        oscillations = 0;
        goingUp = false;

        // Run the robot for a few seconds with the test Kp value.
        while (millis() - start < 3000) {
            bool lineFound, inter, lw, rw;
            int pos = readLine(lineFound, inter, lw, rw);
            error = pos - SETPOINT;
            derivative = error - lastError;
            lastError = error;

            // Count how many times the robot crosses the setpoint.
            if ((derivative > 0 && !goingUp) || (derivative < 0 && goingUp)) {
                goingUp = !goingUp;
                oscillations++;
            }

            // Drive the robot with only the proportional term.
            double correction = (Kp * error);
            int left = BASE_SPEED + correction;
            int right = BASE_SPEED - correction;
            m.motor(3, (left >= 0) ? FORWARD : BACKWARD, abs(left));
            m.motor(4, (right >= 0) ? FORWARD : BACKWARD, abs(right));
        }

        // If we have enough oscillations, we've found our ultimate gain.
        if (oscillations >= 6) {
            Ku = testKp;
            Pu = 600.0; // This is a rough estimate of the period.
            break;
        }
        testKp += 0.1;
    }

    stopMotors();

    // If we found a good Ku, we can calculate the final PID values.
    if (Ku > 0) {
        Kp = 0.6 * Ku;
        Ki = 0.0; // Ki is often not needed for a simple line follower.
        Kd = 0.125 * Ku * Pu / 1000.0;

        Serial.print("Auto-tuned -> Kp: "); Serial.print(Kp);
        Serial.print(" Kd: "); Serial.print(Kd);
        Serial.println(" Ki: 0");

        // Save the new values to EEPROM.
        EEPROM.put(100, Kp);
        EEPROM.put(108, Ki);
        EEPROM.put(116, Kd);
        EEPROM.write(99, 55);
        Serial.println("Saved PID to EEPROM.");
    } else {
        Serial.println("Auto-tune failed to find sustained oscillations.");
    }
}

// A simple helper function to load PID values from EEPROM.
void loadPID() {
    if (EEPROM.read(99) == 55) { // 55 is our magic number for PID data.
        EEPROM.get(100, Kp);
        EEPROM.get(108, Ki);
        EEPROM.get(116, Kd);
        Serial.print("Loaded PID -> ");
        Serial.print("Kp: "); Serial.print(Kp);
        Serial.print(" Ki: "); Serial.print(Ki);
        Serial.print(" Kd: "); Serial.println(Kd);
    }
}

// A helper function to stop both motors at once.
void stopMotors() {
    m.motor(3, BRAKE, 0);
    m.motor(4, BRAKE, 0);
}
