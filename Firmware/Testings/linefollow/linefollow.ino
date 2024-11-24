#include <Arduino.h>

// Motor control pins
const int motorA_pin1 = 16; // Motor A forward
const int motorA_pin2 = 4;  // Motor A backward
const int motorB_pin1 = 18; // Motor B forward
const int motorB_pin2 = 17; // Motor B backward

// IR sensor pins
const int irPins[8] = {14, 27, 26, 25, 33, 32, 35, 34};
const int threshold = 200; // IR threshold

// PID constants
const float kp = 25.0; // Proportional gain
const float ki = 0.0; // Integral gain
const float kd = 2.0; // Derivative gain

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
        sensorValues[i] = analogRead(irPins[i]);
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

void setup() {
    Serial.begin(115200);

    // Initialize motor pins
    pinMode(motorA_pin1, OUTPUT);
    pinMode(motorA_pin2, OUTPUT);
    pinMode(motorB_pin1, OUTPUT);
    pinMode(motorB_pin2, OUTPUT);

    // Initialize IR sensor pins
    for (int i = 0; i < 8; i++) {
        pinMode(irPins[i], INPUT);
    }

    Serial.println("Line-following robot starting...");
}

void loop() {
    line_following_pid_forward();
}
