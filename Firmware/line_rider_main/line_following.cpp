#include <Arduino.h>
#include "line_following.h"
#include "ir.h"
#include "motors.h"
// Motor control pins
// const int motorA_pin1 = 16; // Motor A forward
// const int motorA_pin2 = 4;  // Motor A backward
// const int motorB_pin1 = 18; // Motor B forward
// const int motorB_pin2 = 17; // Motor B backward

// IR sensor pins
const int irArrayPins[8] = {14, 27, 26, 25, 33, 32, 35, 34};
const int threshold = 200; // IR threshold

// PID constants
const float kp = 25.0; // Proportional gain
const float ki = 0.0; // Integral gain
const float kd = 2; // Derivative gain

// PID variables
float integral = 0.0;
float lastError = 0.0;

// Motor speed settings
const int motorSpeed = 120; // Base speed
const int maxSpeed = 255;   // Maximum speed
const int minSpeed = 30;     // Minimum speed

void line_following_pid_forward() {
    // Read sensor values
    int sensorValues[8];
    for (int i = 0; i < 8; i++) {
        sensorValues[i] = analogRead(irArrayPins[i]);
    }

    // Calculate error value
    float error = 0.0;
    for (int i = 0; i < 8; i++) {
        if (sensorValues[i] > threshold) {
            error += (i - 3.275); // Center of sensor array is 3.275 (0 to 7 indices)
        }
    }

    // PID control
    float output = kp * error;               // Proportional term
    integral += error;                       // Integral term
    output += ki * integral;                 // Integral term
    output += kd * (error - lastError);      // Derivative term
    lastError = error;

    // Adjust motor speeds based on PID output
    int leftMotorSpeed = motorSpeed + output;
    int rightMotorSpeed = motorSpeed - output;

    // Ensure motor speeds are within valid range
    leftMotorSpeed = constrain(leftMotorSpeed, minSpeed, maxSpeed);
    rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, maxSpeed);

    // Debugging outputs
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(", Output: ");
    Serial.print(output);
    Serial.print(", Left Speed: ");
    Serial.print(leftMotorSpeed);
    Serial.print(", Right Speed: ");
    Serial.println(rightMotorSpeed);

    // Set motor speeds
    analogWrite(motorA_pin1, leftMotorSpeed); // Left motor forward
    analogWrite(motorA_pin2, 0);             // Left motor backward disabled
    analogWrite(motorB_pin1, rightMotorSpeed); // Right motor forward
    analogWrite(motorB_pin2, 0);             // Right motor backward disabled
}


void rotate_to_line(char direction, int speed, int initialTime) {
    // Initial rotation for a specified time
    if (direction == 'L') {
        turnLeft(speed); // Rotate left
        delay(initialTime);
        stopMotors(); // Stop motors after initial rotation
    } else if (direction == 'R') {
        turnRightOneWheel(speed); // Rotate right
        delay(initialTime);
        stopMotors(); // Stop motors after initial rotation
    }

    // Continue rotating until the line is detected
    while (true) {
        readIRValues();

        // Check the appropriate IR sensor based on the direction
        if ((direction == 'L' && irValues[4] > thresholdsIR[4]) || 
            (direction == 'R' && irValues[5] > thresholdsIR[5])) {
            break; // Exit the loop if the line is detected
        }

        // Continue rotation based on direction
        if (direction == 'L') {
            turnLeft(speed);
        } else if (direction == 'R') {
            turnRightOneWheel(speed);
        }
    }

    stopMotors(); // Ensure motors stop after detecting the line
}



