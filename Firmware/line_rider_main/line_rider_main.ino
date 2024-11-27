// #include <WiFi.h>
// #include <ArduinoOTA.h>
#include <SPI.h>
#include <Wire.h>
#include "display.h"  // Include the display header
#include "motors.h" // motor header
#include "ir.h" // ir header
#include "line_following.h" // line following header



#define BUTTON_1 15  
#define BUTTON_2 0


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int main_section = 0; // 0 button , 1 run 

//  For button Functionalities 
int mode = 0;  // 0-sleep , 1-calibraion , 2-line following
int mode_on =0; // 0 and 1 

// for section of line following 
int section =0;


char line_color = 'B'; // start black line


void setup() {

    Serial.begin(115200);

    pinMode(BUTTON_1, INPUT_PULLUP);  
    pinMode(BUTTON_2, INPUT_PULLUP);  

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));   // no for loop if there is an error in display
    return;
    }

    // Initiaate motor pins
    pinMode(motorA_pin1, OUTPUT);
    pinMode(motorA_pin2, OUTPUT);
    pinMode(motorB_pin1, OUTPUT);
    pinMode(motorB_pin2, OUTPUT);

//   // Configure left encoder pins as input with pull-up
//     pinMode(LEFT_ENCODER_PIN_A, INPUT_PULLUP);
//     pinMode(LEFT_ENCODER_PIN_B, INPUT_PULLUP);

//     // Configure right encoder pins as input with pull-up
//     pinMode(RIGHT_ENCODER_PIN_A, INPUT_PULLUP);
//     pinMode(RIGHT_ENCODER_PIN_B, INPUT_PULLUP);

//     // Attach interrupts for left encoder
//     attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN_A), handleLeftEncoderA, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN_B), handleLeftEncoderB, CHANGE);

//     // Attach interrupts for right encoder
//     attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN_A), handleRightEncoderA, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN_B), handleRightEncoderB, CHANGE);

    // Initialize IR sensor pins
    for (int i = 0; i < 10; i++) {
        pinMode(irPins[i], INPUT);
    }

    loadThresholds();

    // xTaskCreate(readIRTask, "Read IR Task", 1000, NULL, 1, NULL); // Create a task for reading IR values

    delay(100);
    displayText("Welcome");
    delay(2000);

}

void loop() {


    if (main_section==0){

    int buttonState1 = digitalRead(BUTTON_1); // Read the state of the first button
    int buttonState2 = digitalRead(BUTTON_2); // Read the state of the second button

    if (buttonState1 == LOW) {
        mode_on = 0;
        mode = (mode == 2) ? 0 : mode + 1; // Cycle through modes 0, 1, 2
    }

    if (buttonState2 == LOW) {
        mode_on = (mode_on == 1) ? 0 : 1; // Cycle through mode_on 0, 1
    }

    if (mode == 0) {
        displayText("Sleep mode");

        //serial monitor shoe threshold values
        // for (int i = 0; i < 10; i++) {
        //     Serial.print(thresholdsIR[i]);
        //     Serial.print(" ");
        // }
            Serial.println("");

        if (mode_on == 1) {
            readIRValues();
            displayWhiteBlack();
            delay(1000);
        }
    } else if (mode == 1) {
        if (mode_on == 0) {
            displayText("Calibration mode");
        } else if (mode_on == 1) {
            displayText("Calibrating");
            autoCalibrateIR();
            //code for calibration
            displayText("Calibrate done");
            delay(2000);
            displayWhiteBlack();


            mode_on=0;
        }
    } else if (mode == 2) {

        if (mode_on == 0) {
            displayText("Line following mode");

        } else if (mode_on == 1) {
            main_section = 1;
        }

    }

      delay(200); // Delay for readability

}

    else if (main_section == 1) {

        // line_following_pid_forward();

        readIRValues();

        if (right_junction(line_color)) {
            stopMotors();
            displayText("Right Junction");
            delay(1000);
            display.clearDisplay();
            rotate_to_line('R', 100, 500); 
        } 


        else {

            line_following_pid_forward();
        }

    }

    delay(50); // Delay for smooth operation




}