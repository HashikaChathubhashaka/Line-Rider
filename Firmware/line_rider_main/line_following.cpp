#include "line_following.h"
#include "ir.h"
#include "motors.h"
#include <Arduino.h> 


// Speed settings
int baseSpeed = 150;  // Base speed for motors
int maxSpeed = 255;   // Maximum speed for motors

// PID constants
float Kp = 2.0;  // Proportional gain
float Ki = 0.5;  // Integral gain
float Kd = 1.0;  // Derivative gain

// PID variables
float previousError = 0;
float integral = 0;


// int irValues[10];



// Function to calculate line position using threshold
int calculatePosition() {
    int weightedSum = 0;
    int totalValue = 0;

    for (int i = 0; i < 10; i++) {
        int value = irValues[i] > thresholdsIR[i] ? 1 : 0;  // Compare with individual threshold
        weightedSum += value * (i + 1);  // Weight by sensor position
        totalValue += value;
    }

    if (totalValue == 0) {
        return -1;  // Line not detected
    }

    return weightedSum / totalValue;  // Return line position (1-10)
}


// Function to follow the line using PID control
void lineFollowPID() {
    readIRValues();
    int position = calculatePosition();

    if (position == -1) {
        stopMotors();
        return;  // Line not detected
    }

    // Calculate error
    float error = position - 5;  // Target position is the center of the array (5)

    // PID control
    integral += error;
    float derivative = error - previousError;
    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    previousError = error;

    // Adjust motor speeds
    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    // Constrain speeds to valid range
    leftSpeed = constrain(leftSpeed, 0, maxSpeed);
    rightSpeed = constrain(rightSpeed, 0, maxSpeed);

    setMotorSpeeds(leftSpeed, rightSpeed);
}


