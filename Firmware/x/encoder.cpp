#include <Encoder.h>
#include <Arduino.h>
#include "motors.h"
#include "encoder.h"


// Encoder objects (Assuming you have two encoders for left and right wheels)
Encoder leftEncoder(LEFT_ENCODER_PIN_A, LEFT_ENCODER_PIN_B);
Encoder rightEncoder(RIGHT_ENCODER_PIN_A, RIGHT_ENCODER_PIN_B);

void moveForwardDistance(float distanceInCm, int speed) {
    long targetPulses = distanceInCm * pulsesPerCm;

    leftEncoder.write(0);  // Reset encoder counts
    rightEncoder.write(0);

    while (true) {
        long leftCount = abs(leftEncoder.read());
        long rightCount = abs(rightEncoder.read());

        // Check if both encoders have reached the target
        if (leftCount >= targetPulses && rightCount >= targetPulses) {
            stopMotors();  // Stop motors when target is reached
            break;
        }

        // Adjust motor speed to maintain straight movement
        if (leftCount < rightCount) {
            analogWrite(motorA_pin1, speed + 10);  // Slightly increase left motor speed
            analogWrite(motorB_pin1, speed);
        } else if (rightCount < leftCount) {
            analogWrite(motorA_pin1, speed);
            analogWrite(motorB_pin1, speed + 10);  // Slightly increase right motor speed
        } else {
            analogWrite(motorA_pin1, speed);
            analogWrite(motorB_pin1, speed);
        }
    }
}