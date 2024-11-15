#include "motors.h"
#include <Arduino.h>  // Include Arduino library for analogWrite

int speed = 170;

void moveForward() {
    analogWrite(motorA_pin1, speed);
    analogWrite(motorA_pin2, 0);
    analogWrite(motorB_pin1, speed);
    analogWrite(motorB_pin2, 0);
}
