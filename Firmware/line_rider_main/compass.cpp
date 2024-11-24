#include <QMC5883LCompass.h>
#include <Arduino.h>
#include "motors.h"
#include "compass.h"
// Initialize the QMC5883L compass
QMC5883LCompass compass;

// Function to initialize the compass
void initCompass() {
    compass.init();
    compass.setCalibration(-100, 100, -100, 100, 0, 0); // Adjust calibration values as needed

}

// Function to read the current heading
float getHeading() {
    compass.read();
    float heading = compass.getAzimuth();
    return heading;
}

// Normalize angle to 0-360 range
float normalizeAngle(float angle) {
    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;
    return angle;
}

// Function to rotate robot to a specific angle
void rotateToAngle(float angle, int turnSpeed) {
    float initialHeading = getHeading();
    float targetHeading = normalizeAngle(initialHeading + angle);

    while (true) {
        float currentHeading = getHeading();
        float error = targetHeading - currentHeading;

        // Normalize error to be between -180 and 180
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        if (abs(error) < 2) {  // Error margin of ±2 degrees
            stopMotors();
            break;
        }

        if (error > 0) {
            turnRight(turnSpeed);
        } else {
            turnLeft(turnSpeed);
        }
    }
}
