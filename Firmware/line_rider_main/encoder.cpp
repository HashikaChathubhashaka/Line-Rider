#include <Arduino.h>
#include "motors.h"
#include "encoder.h"


// Volatile variables to store encoder counts
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Interrupt service routines for left encoder
void handleLeftEncoderA() {
  if (digitalRead(LEFT_ENCODER_PIN_A) == digitalRead(LEFT_ENCODER_PIN_B)) {
    leftEncoderCount++; // Increment for one direction
  } else {
    leftEncoderCount--; // Decrement for the other direction
  }
}

void handleLeftEncoderB() {
  if (digitalRead(LEFT_ENCODER_PIN_A) != digitalRead(LEFT_ENCODER_PIN_B)) {
    leftEncoderCount++; // Increment for one direction
  } else {
    leftEncoderCount--; // Decrement for the other direction
  }
}

// Interrupt service routines for right encoder
void handleRightEncoderA() {
  if (digitalRead(RIGHT_ENCODER_PIN_A) == digitalRead(RIGHT_ENCODER_PIN_B)) {
    rightEncoderCount++; // Increment for one direction
  } else {
    rightEncoderCount--; // Decrement for the other direction
  }
}

void handleRightEncoderB() {
  if (digitalRead(RIGHT_ENCODER_PIN_A) != digitalRead(RIGHT_ENCODER_PIN_B)) {
    rightEncoderCount++; // Increment for one direction
  } else {
    rightEncoderCount--; // Decrement for the other direction
  }
}

// Constants for movement calculations
const float wheelDiameter = 3.0;        // Diameter of the wheel in cm
const int encoderPPR = 7;               // Pulses Per Revolution (PPR) of the encoder
const float gearRatio = 150.0;           // Gear ratio of the motor
const float wheelCircumference = wheelDiameter * 3.14159;  // Circumference in cm
const float pulsesPerCm = (encoderPPR * gearRatio) / wheelCircumference;  // Pulses per cm traveled


void moveForwardDistance(float distanceInCm, int speed) {
  // Calculate the target encoder counts for the given distance
  long targetCounts = distanceInCm * pulsesPerCm;

  // Reset encoder counts
  noInterrupts();
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  interrupts();
  
  // // Give a brief "kick" to the motors
  // int kickStartSpeed = min(speed + 100, 255); // Increase speed for the start
  // analogWrite(motorA_pin1, kickStartSpeed);
  // analogWrite(motorA_pin2, 0);
  // analogWrite(motorB_pin1, kickStartSpeed);
  // analogWrite(motorB_pin2, 0);

  // delay(100); // Run motors
  // Set motors to move forward
  analogWrite(motorA_pin1, speed);
  analogWrite(motorA_pin2, 0);
  analogWrite(motorB_pin1, speed);
  analogWrite(motorB_pin2, 0);

  // Monitor encoder counts
  while (true) {
    noInterrupts();
    long leftCount = leftEncoderCount;
    long rightCount = rightEncoderCount;
    interrupts();

    // Stop when both encoders reach the target counts
    if (leftCount >= targetCounts && rightCount >= targetCounts) {
      break;
    }

    // Adjust motor speeds if one motor is ahead
    if (leftCount > rightCount) {
      analogWrite(motorA_pin1, speed - 10); // Slow down left motor
    } else if (rightCount > leftCount) {
      analogWrite(motorB_pin1, speed - 10); // Slow down right motor
    } else {
      analogWrite(motorA_pin1, speed);
      analogWrite(motorB_pin1, speed);
    }
  }

  // Stop both motors
  stopMotors();
}

void rotateByAngle(float angle, bool clockwise, int baseSpeed) {
    // Robot configuration
    const float wheelBase = 13.0; // Distance between wheels in cm (adjust for your robot)
    const float pulsesPerCm = 10; // Pulses per cm (adjust for your encoders)
    const float Kp = 4, Ki = 0.01, Kd = 0.05; // PID constants (tune these for your robot)

    // Calculate the distance each wheel needs to travel for the desired rotation
    float arcLength = (angle / 360.0) * (3.14159 * wheelBase); // Arc length for rotation
    long targetCounts = arcLength * pulsesPerCm;

    // Reset encoder counts
    noInterrupts();
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    interrupts();

    // Initialize PID variables
    long previousError = 0;
    long integral = 0;

    while (true) {
        // Read encoder counts
        noInterrupts();
        long leftCount = abs(leftEncoderCount);  // Use absolute values for comparison
        long rightCount = abs(rightEncoderCount);
        interrupts();

        // Calculate error (difference between left and right encoder counts)
        long error = leftCount - rightCount;

        // PID calculations
        integral += error;                      // Accumulate integral
        long derivative = error - previousError; // Calculate derivative
        previousError = error;

        int correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

        // Calculate motor speeds
        int leftSpeed = baseSpeed;
        int rightSpeed = baseSpeed;

        if (clockwise) {
            leftSpeed += correction;  // Adjust left motor for correction
            rightSpeed -= correction; // Adjust right motor for correction
        } else {
            leftSpeed -= correction;  // Adjust left motor for correction
            rightSpeed += correction; // Adjust right motor for correction
        }

        // Constrain motor speeds to valid range (0 to 255)
        leftSpeed = constrain(leftSpeed, 0, 255);
        rightSpeed = constrain(rightSpeed, 0, 255);

        // Set motor speeds
        analogWrite(motorA_pin1, clockwise ? leftSpeed : 0);
        analogWrite(motorA_pin2, clockwise ? 0 : leftSpeed);
        analogWrite(motorB_pin1, clockwise ? 0 : rightSpeed);
        analogWrite(motorB_pin2, clockwise ? rightSpeed : 0);

        // Check if target counts are reached
        if (leftCount >= targetCounts && rightCount >= targetCounts) {
            break;
        }

        delay(10); // Small delay for stability
    }

    // Stop motors
    stopMotors();
}

