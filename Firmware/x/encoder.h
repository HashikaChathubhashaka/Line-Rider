#ifndef ENCODER_H
#define ENCODER_H

// Left encoder pins
const int LEFT_ENCODER_PIN_A = 23;  // GPIO pin for left encoder A
const int LEFT_ENCODER_PIN_B = 19;  // GPIO pin for left encoder B

// Right encoder pins
const int RIGHT_ENCODER_PIN_A = 13;  // GPIO pin for right encoder A
const int RIGHT_ENCODER_PIN_B = 5;   // GPIO pin for right encoder B

// Constants for movement calculations
const float wheelDiameter = 3.0;        // Diameter of the wheel in cm
const int encoderPPR = 7;               // Pulses Per Revolution (PPR) of the encoder
const float gearRatio = 20.0;           // Gear ratio of the motor
const float wheelCircumference = wheelDiameter * 3.14159;  // Circumference in cm
const float pulsesPerCm = (encoderPPR * gearRatio) / wheelCircumference;  // Pulses per cm traveled

// Function declarations
void moveForwardDistance(float distanceInCm, int speed);

#endif // ENCODER_H
