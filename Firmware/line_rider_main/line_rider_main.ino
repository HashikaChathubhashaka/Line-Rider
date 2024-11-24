#include <SPI.h>
#include <Wire.h>


#include "display.h"  // Include the display header
#include "motors.h" // motor header
#include "ir.h" // ir header
#include "line_following.h" // line following header
// #include "encoder.h" // encoder header
#include "compass.h" // compass header

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


// ##############################
const int LEFT_ENCODER_PIN_A = 23;  // GPIO pin for left encoder A
const int LEFT_ENCODER_PIN_B = 19;  // GPIO pin for left encoder B

// Right encoder pins
const int RIGHT_ENCODER_PIN_A = 13;  // GPIO pin for right encoder A
const int RIGHT_ENCODER_PIN_B = 5;   // GPIO pin for right encoder B


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



// #############################


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

  // Configure left encoder pins as input with pull-up
    pinMode(LEFT_ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(LEFT_ENCODER_PIN_B, INPUT_PULLUP);

    // Configure right encoder pins as input with pull-up
    pinMode(RIGHT_ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN_B, INPUT_PULLUP);

    // Attach interrupts for left encoder
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN_A), handleLeftEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN_B), handleLeftEncoderB, CHANGE);

    // Attach interrupts for right encoder
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN_A), handleRightEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN_B), handleRightEncoderB, CHANGE);

    // Initialize IR sensor pins
    for (int i = 0; i < 10; i++) {
        pinMode(irPins[i], INPUT);
    }

    loadThresholds();

    delay(100);
    displayText("Welcome");
    delay(200);
    // rotateByAngle(50,true,90);
    // moveForward(100);
    // delay(400);
    // stopMotors();

}

void loop() {
    int buttonState1 = digitalRead(BUTTON_1); // Read the state of the first button
    int buttonState2 = digitalRead(BUTTON_2); // Read the state of the second button

    if (buttonState1 == LOW) {
        // Serial.println("Button 1 Pressed");
        mode_on = 0;
        mode = (mode == 2) ? 0 : mode + 1; // Cycle through modes 0, 1, 2
    } else {
        // Serial.println("Button 1 Not Pressed");
    }

    if (buttonState2 == LOW) {
        // Serial.println("Button 2 Pressed");
        mode_on = (mode_on == 1) ? 0 : 1; // Cycle through mode_on 0, 1
    } else {
        // Serial.println("Button 2 Not Pressed");
    }

    if (mode == 0) {
        displayText("Sleep mode");
        //serial monitor shoe threshold values
        for (int i = 0; i < 10; i++) {
            Serial.print(thresholdsIR[i]);
            Serial.print(" ");
        }
        
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
            delay(2000);
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
            // displayText("Following");
            // testmotors();
            while (true) {
                readIRValues();
                
                
                if (irValues[1] > thresholdsIR[1] && irValues[5]>thresholdsIR[5] && irValues[8] > thresholdsIR[8]) {
                    stopMotors();
                    displayText("BL detected");
                    delay(1000);
                    displayText("distance");
                    unsigned long startTime = millis(); // Record the current time
                    unsigned long runDuration = 100;   // Time to run in milliseconds (e.g., 5000ms = 5 seconds)

                    while (millis() - startTime < runDuration) {
                        line_following_pid_forward();
}
                    stopMotors();
                    displayText("rotate");
                    rotateByAngle(40,true,90);
                }
          
                else {
                  line_following_pid_forward();
                }
                
            }
            // moveForwardDistance(5, 170);
            //moveForwardDistance(10, 100);
        
            mode_on=0;
        }

    }

    delay(200); // Delay for readability
}
