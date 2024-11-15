#include <SPI.h>
#include <Wire.h>


#include "display.h"  // Include the display header
#include "motors.h" // motor header



#define BUTTON_1 15  
#define BUTTON_2 0

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


//  For button Functionalities 
int mode = 0;  // 0-sleep , 1-calibraion , 2-line following
int mode_on =0; // 0 and 1 

// for section of line following 
int section =0;
int subsection =0;



void setup() {

    Serial.begin(115200);
    pinMode(BUTTON_1, INPUT_PULLUP);  
    pinMode(BUTTON_2, INPUT_PULLUP);  

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));   // no for loop if there is an error in display
    return;
    }

    pinMode(motorA_pin1, OUTPUT);
    pinMode(motorA_pin2, OUTPUT);
    pinMode(motorB_pin1, OUTPUT);
    pinMode(motorB_pin2, OUTPUT);

    delay(100);
    displayText("Welcome");
    delay(2000);

}

void loop() {
    int buttonState1 = digitalRead(BUTTON_1); // Read the state of the first button
    int buttonState2 = digitalRead(BUTTON_2); // Read the state of the second button

    if (buttonState1 == LOW) {
        Serial.println("Button 1 Pressed");
        mode_on = 0;
        mode = (mode == 2) ? 0 : mode + 1; // Cycle through modes 0, 1, 2
    } else {
        Serial.println("Button 1 Not Pressed");
    }

    if (buttonState2 == LOW) {
        Serial.println("Button 2 Pressed");
        mode_on = (mode_on == 1) ? 0 : 1; // Cycle through mode_on 0, 1
    } else {
        Serial.println("Button 2 Not Pressed");
    }

    if (mode == 0) {
        displayText("Sleep mode");
    } else if (mode == 1) {
        if (mode_on == 0) {
            displayText("Calibration mode");
        } else if (mode_on == 1) {
            displayText("Calibrating");
            delay(2000);
            //code for calibration
            displayText("Calibrate done");
            delay(2000);

            mode_on=0;
        }
    } else if (mode == 2) {

        if (mode_on == 0) {
            displayText("Line following mode");
        } else if (mode_on == 1) {
            displayText("Following");
            delay(4000);
            // Code for line following
            mode_on=0;
        }

    }

    delay(200); // Delay for readability
}
