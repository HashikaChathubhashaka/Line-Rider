#ifndef ENCODER_H
#define ENCODER_H


const int LEFT_ENCODER_PIN_A = 23;  // GPIO pin for left encoder A
const int LEFT_ENCODER_PIN_B = 19;  // GPIO pin for left encoder B

// Right encoder pins
const int RIGHT_ENCODER_PIN_A = 13;  // GPIO pin for right encoder A
const int RIGHT_ENCODER_PIN_B = 5;   // GPIO pin for right encoder B


void moveForwardDistance(float distanceInCm, int speed);
void rotateByAngle(float angle, bool clockwise, int baseSpeed);
void handleLeftEncoderA();
void handleLeftEncoderB();
void handleRightEncoderA();
void handleRightEncoderB();

#endif