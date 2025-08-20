# PID-LFR
High-Performance PID Line Follower (Competition Build v16.2)
This repository contains the Arduino source code for a high-speed, 11-sensor line-following robot. It's designed for competitive tracks and uses a finely-tuned PID (Proportional-Integral-Derivative) control algorithm coupled with a state machine to handle complex track elements like sharp turns and intersections.

The code is built for performance and requires careful tuning to match your specific robot's hardware, weight, and motor characteristics.

Key Features
PID Control: Implements a PID controller for smooth and aggressive line tracking at high speeds.

State Machine: A multi-state system to intelligently handle different scenarios:

FOLLOWING: Normal PID line following.

SHARP_TURN: Aggressive turning logic for tight corners, triggered by wing sensors or rapid error changes.

CROSSING_INTERSECTION: Drives straight for a fixed duration to navigate intersections without getting lost.

RECOVERY: A backup-and-turn maneuver for when the line is completely lost.

Dynamic Speed Control: Automatically reduces speed in curves and accelerates on straightaways to maintain stability and maximize velocity.

Onboard Calibration: A simple, one-button calibration routine to read sensor values for the track surface and line, saving the results to EEPROM.

EEPROM Storage: Saves calibration data so you don't have to re-calibrate every time the robot powers on.

Hardware Requirements
Microcontroller: Arduino Mega (or compatible board with enough analog pins)

Motor Driver: L293D Motor Shield (or a similar dual-channel motor driver)

Sensors: An array of 11 analog IR/QTR sensors.

Motors: Two DC motors (e.g., N20 micro gear motors).

Chassis & Power: A robot chassis, wheels, and an appropriate power source (e.g., LiPo battery) capable of supplying sufficient current without voltage drops.

Software Dependencies
This code requires the following Arduino library:

MotorDriver.h: A custom or third-party library for controlling the L293D shield. Ensure you have this library installed.

EEPROM.h: A standard library included with the Arduino IDE.

How to Use
1. Initial Setup
Upload the Code: Flash the .ino file to your Arduino Mega.

Motor Direction: Check if your motors run in the correct direction. If the robot turns the wrong way, flip the INVERT_LEFT_MOTOR or INVERT_RIGHT_MOTOR constants at the top of the file from false to true.

2. Sensor Calibration (Very Important!)
You must calibrate the sensors on the track you will be using.

Place the robot on the track.

Power it on.

Press and hold the start button for 2 seconds.

The robot will begin rotating in place for about 2.5 seconds. During this time, manually sweep the sensor array back and forth over the line to ensure every sensor sees both the black line and the white surface.

The robot will stop, and the new calibration values will be saved automatically.

3. Operation
Short Press: Press and release the button to Start or Stop the robot.

Long Press (2s): Press and hold the button to enter Calibration Mode.

PID Tuning Guide
This is the most critical step for achieving high performance. The default PID values (Kp, Ki, Kd) are a starting point and will not be optimal for your robot. Follow these steps methodically.

Step 1: Tune Kp (Proportional Gain)
Kp determines how strongly the robot reacts to being off-center.

In the code, set Kd to 0. We only want to see Kp's effect for now.

Start with the provided Kp value (e.g., 3.5).

Place the robot on the line and start it.

Goal: Find the Kp value that makes the robot wobble (oscillate) quickly and consistently from side to side as it follows the line.

If the robot is sluggish and drives off the line on gentle curves, INCREASE Kp.

If the robot shakes violently and uncontrollably, DECREASE Kp.

Adjust Kp in small steps (e.g., by 0.2 or 0.3) until you find the value that causes a fast, sustained wobble. This is your "Ultimate Gain". Note it down.

Step 2: Tune Kd (Derivative Gain)
Kd acts as a dampener, smoothing out the wobble caused by Kp. It anticipates the future position of the line based on the current rate of change.

Set Kp to about half of the "Ultimate Gain" you found in Step 1.

Start with a small Kd value (e.g., 1.0).

Goal: Find the Kd value that eliminates the wobble and makes the robot follow the line smoothly.

If the robot still wobbles, INCREASE Kd.

If the robot becomes sluggish again and overshoots the line on turns, you have too much Kd. DECREASE Kd.

Adjust Kd until the wobble is gone and the robot is stable and responsive on both straights and curves.

Step 3: Ki (Integral Gain)
For most fast-paced tracks, Ki is not necessary and can cause instability ("integral windup"). It's designed to correct for small, persistent, long-term errors.

Leave Ki at 0.0 or a very small value (0.0005) unless you notice the robot consistently driving slightly off-center on very long, straight sections of the track.

Troubleshooting
Robot resets or acts erratically: If you see the "ROBOT BOOT / RESET" message in the Serial Monitor multiple times during a run, you have a power issue. Your battery cannot supply enough current, causing the Arduino to "brownout" and reset. Use a better battery or add a large capacitor across the power rails.

Poor turning performance: This is almost always a PID tuning issue. Re-run the tuning process carefully.

Robot gets lost easily: Your sensor calibration may be poor. Make sure you expose all sensors to both the line and the background during the calibration sweep. Different lighting conditions require re-calibration.
