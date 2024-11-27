#include "ir.h"
#include "motors.h"
#include <Arduino.h>  // Include Arduino library for analogWrite
#include <Preferences.h>


Preferences preferences;

int thresholdsIR[10]; // Threshold values for IR sensors
int irValues[10]; // current IR values



// Save thresholds to NVS
void saveThresholds() {
    preferences.begin("thresholds", false); // Open storage in RW mode
    for (int i = 0; i < 10; i++) {
        String key = "thresh" + String(i);  // Generate the key
        preferences.putInt(key.c_str(), thresholdsIR[i]); // Use .c_str() to convert String to const char*
    }
    preferences.end(); // Close storage
}

// Load thresholds from NVS
void loadThresholds() {
    preferences.begin("thresholds", true); // Open storage in read-only mode
    for (int i = 0; i < 10; i++) {
        String key = "thresh" + String(i);  // Generate the key
        thresholdsIR[i] = preferences.getInt(key.c_str(), 512); // Default value is 512
    }
    preferences.end(); // Close storage
}

void readIRValues() {
    for (int i = 0; i < 10; i++) {
        irValues[i] = analogRead(irPins[i]);
    }
}


// calibration function for IR sensors
void autoCalibrateIR() { 
    int minValues[10]; // Minimum values for each IR sensor
    int maxValues[10]; // Maximum values for each IR sensor

    // Initialize min and max values
    for (int i = 0; i < 10; i++) {
        minValues[i] = 1023; // Maximum ADC value
        maxValues[i] = 0;    // Minimum ADC value
    }

    // Rotate the robot left and right
    for (int t = 0; t < 1500; t++) { // Run for 200 cycles (~adjust as needed)
        readIRValues();
        for (int i = 0; i < 10; i++) {
            int sensorValue = irValues[i];
            if (sensorValue < minValues[i]) minValues[i] = sensorValue;
            if (sensorValue > maxValues[i]) maxValues[i] = sensorValue;
        }

        // // Move robot left and right alternately
        if (t < 1000) {
            turnLeft(130); // Rotate left at speed 80
        } else {
            turnRight(130); // Rotate right at speed 80
        }
        delay(10); // Delay for smooth movement and reading
    }

    stopMotors(); // Stop motors after calibration
    // Calculate thresholds for each sensor
    for (int i = 0; i < 10; i++) {
        thresholdsIR[i] = (minValues[i] + maxValues[i]) / 2; // Midpoint as threshold
    }
    saveThresholds(); // Save thresholds to NVS
}



// Detections

bool right_junction(char lineColor) {
    if (lineColor == 'B') { // Black lines
        if (irValues[5] > thresholdsIR[5] && irValues[8] > thresholdsIR[8]) {
            return true;
        }
    } else if (lineColor == 'W') { // White lines
        if (irValues[5] < thresholdsIR[5] && irValues[8] < thresholdsIR[8]) {
            return true;
        }
    }
    return false;
}

bool left_junction(char lineColor) {
    if (lineColor == 'B') { // Black lines
        if (irValues[1] > thresholdsIR[1] && irValues[4] > thresholdsIR[4]) {
            return true;
        }
    } else if (lineColor == 'W') { // White lines
        if (irValues[1] < thresholdsIR[1] && irValues[4] < thresholdsIR[4]) {
            return true;
        }
    }
    return false;
}

bool mid_IR(char lineColor) {
    if (lineColor == 'B') { // Black lines
        if (irValues[5] > thresholdsIR[5]) {
            return true;
        }
    } else if (lineColor == 'W') { // White lines
        if (irValues[5] < thresholdsIR[5] && irValues[5] < thresholdsIR[5]) {
            return true;
        }
    }
    return false;
}

bool dead_end_or_dotted(char lineColor) {
    if (lineColor == 'B') { // White lines - black dead end
        if (irValues[1] < thresholdsIR[1] && irValues[8] < thresholdsIR[8]) {
            return true;
        }
    } else if (lineColor == 'W') { // Black lines - white dead end
        if (irValues[1] > thresholdsIR[1] && irValues[8] > thresholdsIR[8]) {
            return true;
        }
    }
    return false;
}


bool all_oposite_color(char lineColor) {
    if (lineColor == 'B') { // Black lines
        if (irValues[1] < thresholdsIR[1] && irValues[2] < thresholdsIR[2] && irValues[3] < thresholdsIR[3] && irValues[4] < thresholdsIR[4] && irValues[5] < thresholdsIR[5] && irValues[6] < thresholdsIR[6] && irValues[7] < thresholdsIR[7]) {
            return true;
        }
    } else if (lineColor == 'W') { // White lines
        if (irValues[1] > thresholdsIR[1] && irValues[2] > thresholdsIR[2] && irValues[3] > thresholdsIR[3] && irValues[4] > thresholdsIR[4] && irValues[5] > thresholdsIR[5] && irValues[6] > thresholdsIR[6] && irValues[7] > thresholdsIR[7]) {
            return true;
        }
    }
    return false;
}