#include <QMC5883LCompass.h>

// Compass and motor pins
QMC5883LCompass compass;
const int motorA_pin1 = 16;
const int motorA_pin2 = 4;
const int motorB_pin1 = 18;
const int motorB_pin2 = 17;

// Function to initialize motors
void initMotors() {
    pinMode(motorA_pin1, OUTPUT);
    pinMode(motorA_pin2, OUTPUT);
    pinMode(motorB_pin1, OUTPUT);
    pinMode(motorB_pin2, OUTPUT);
}

// Function to stop motors
void stopMotors() {
    analogWrite(motorA_pin1, 0);
    analogWrite(motorA_pin2, 0);
    analogWrite(motorB_pin1, 0);
    analogWrite(motorB_pin2, 0);
}

// Function to rotate the robot
void rotateRobot(float angle, bool clockwise) {
    // Get the current heading
    compass.read();
    float currentHeading = compass.getAzimuth();

    // Calculate the target heading
    float targetHeading = clockwise ? currentHeading - angle : currentHeading + angle;

    // Normalize targetHeading to the range [0, 360)
    if (targetHeading < 0) targetHeading += 360;
    if (targetHeading >= 360) targetHeading -= 360;

    // Debugging: Print initial readings
    Serial.println("Starting rotation...");
    Serial.print("Initial Heading: ");
    Serial.println(currentHeading);
    Serial.print("Target Heading: ");
    Serial.println(targetHeading);

    // Rotate the motors in the desired direction
    if (clockwise) {
        // Rotate right
        analogWrite(motorA_pin1, 0);
        analogWrite(motorA_pin2, 100);
        analogWrite(motorB_pin1, 100);
        analogWrite(motorB_pin2, 0);

    } else {
        // Rotate left

        analogWrite(motorA_pin1, 100);
        analogWrite(motorA_pin2, 0);
        analogWrite(motorB_pin1, 0);
        analogWrite(motorB_pin2, 100);
    }

    // Rotate until the target heading is reached
    while (true) {
        compass.read();
        float currentHeading = compass.getAzimuth();

        // Calculate the error considering angle wrapping
        float error = targetHeading - currentHeading;

        // Normalize error to the range [-180, 180)
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        // Debugging: Print current heading and error
        Serial.print("Current Heading: ");
        Serial.println(currentHeading);
        Serial.print("Error: ");
        Serial.println(error);

        // Stop if within acceptable range (e.g., 3 degrees)
        if (abs(error) < 3) break;

        delay(50); // Small delay to avoid excessive serial output
    }

    // Stop the motors
    stopMotors();

    // Debugging: Print final heading
    compass.read();
    float finalHeading = compass.getAzimuth();
    Serial.println("Rotation complete.");
    Serial.print("Final Heading: ");
    Serial.println(finalHeading);
}


void setup() {
    Serial.begin(115200);
    compass.init();             // Initialize compass
    initMotors();               // Initialize motors
    delay(2000);

    // Rotate robot 45 degrees to the right
    rotateRobot(90, true); //  Clockwise

    delay(2000);

    // // Rotate robot 45 degrees to the left
    // rotateRobot(45, false); // Counter-clockwise
}

void loop() {
    // No actions in loop
}
