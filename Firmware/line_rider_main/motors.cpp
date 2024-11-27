#include "motors.h"
#include <Arduino.h>  // Include Arduino library for analogWrite

int normal_speed = 170;

void moveForward(int speed) {
    analogWrite(motorA_pin1, speed);
    analogWrite(motorA_pin2, 0);
    analogWrite(motorB_pin1, speed);
    analogWrite(motorB_pin2, 0);
}

void moveBackward(int speed) {
    analogWrite(motorA_pin1, 0);
    analogWrite(motorA_pin2, speed);
    analogWrite(motorB_pin1, 0);
    analogWrite(motorB_pin2, speed);
}

void turnLeft(int speed) {
    analogWrite(motorA_pin1, 0);
    analogWrite(motorA_pin2, speed);
    analogWrite(motorB_pin1, speed);
    analogWrite(motorB_pin2, 0);
}

void turnLeftOneWheel(int speed) {
    analogWrite(motorA_pin1, 0);
    analogWrite(motorA_pin2, 0);
    analogWrite(motorB_pin1, speed);
    analogWrite(motorB_pin2, 0);
}

void turnRightOneWheel(int speed) {
    analogWrite(motorA_pin1, speed);
    analogWrite(motorA_pin2, 0);
    analogWrite(motorB_pin1, 0);
    analogWrite(motorB_pin2, 0);
}


void turnRight(int speed) {
    analogWrite(motorA_pin1, speed);
    analogWrite(motorA_pin2, 0);
    analogWrite(motorB_pin1, 0);
    analogWrite(motorB_pin2, speed);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    analogWrite(motorA_pin1, leftSpeed);
    analogWrite(motorA_pin2, 0);
    analogWrite(motorB_pin1, rightSpeed);
    analogWrite(motorB_pin2, 0);
}

void stopMotors() {
    analogWrite(motorA_pin1, 0);
    analogWrite(motorA_pin2, 0);
    analogWrite(motorB_pin1, 0);
    analogWrite(motorB_pin2, 0);
}


void testmotors() {

    analogWrite(motorA_pin1, 100);
    analogWrite(motorA_pin2, 0);
    analogWrite(motorB_pin1, 100);
    analogWrite(motorB_pin2, 0);

}


