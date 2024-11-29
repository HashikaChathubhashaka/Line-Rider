#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels



Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Motor control pins
const int motorA_pin1 = 16; // Motor A forward
const int motorA_pin2 = 4;  // Motor A backward
const int motorB_pin1 = 18; // Motor B forward
const int motorB_pin2 = 17; // Motor B backward

// IR sensor pins
const int irPins[10] = {39,14,27, 26, 25, 33, 32, 35, 34,36};
const int irArrayPins[8] = {14,27, 26, 25, 33, 32, 35, 34};

const int threshold = 200; // IR threshold  for "B"
// const int threshold = 300; // IR threshold for "w"
const int thresholdRight = 300; 
const int thresholdLeft = 300;

// // B" PID constants
const float kp = 25.0; // Proportional gain
const float ki = 0.0; // Integral gain
const float kd = 2.0; // Derivative gain

// W" PID constants
// const float kp = 8.0; // Proportional gain
// const float ki = 0.0; // Integral gain
// const float kd = 0.5; // Derivative gain

// PID variables
float integral = 0.0;
float lastError = 0.0;


// Motor speed settings
const int motorSpeed = 120; // Base speed 120
const int maxSpeed = 255;   // Maximum speed 255
const int minSpeed = 30;     // Minimum speed 30

int irValues[10];


char color = 'B';

void toggleColor(char &color) {
  if (color == 'B') {
    color = 'W';  // Change from Black ('B') to White ('W')
  } else if (color == 'W') {
    color = 'B';  // Change from White ('W') to Black ('B')
  }
}




void displayText(const String &text) {
    display.clearDisplay();
    display.setRotation(2);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(5, 10);
    display.println(text);
    display.display();
}


// void line_following_pid_forward(char lineColor) {
//     readIRValues();  // Function to read IR sensor values into irValues array

//     // Define weights for each sensor, assuming 7 sensors are used
//     int weights[10] = {8,6, 4, 2, 0.5, -0.5, -2, -4, -6,-8};  // Adjust as needed

//     // Calculate the error value using weighted sensor readings
//     float error = 0.0;
//     // int activeSensors = 0;  // Track the number of sensors detecting the line

//     for (int i = 0; i < 10; i++) {
//         bool onLine = (lineColor == 'W' && irValues[i] > threshold) || 
//                       (lineColor == 'B' && irValues[i] > threshold);

//         if (onLine) {
//             error += weights[i];  // Accumulate error using sensor weights
//             // activeSensors++;
//         }
//     }

//     // // Normalize the error by the number of active sensors if any detected
//     // if (activeSensors > 0) {
//     //     error /= activeSensors;
//     // }

//     // PID Control Calculation
//     float output = kp * error;                // Proportional term
//     integral += error;                        // Integral term
//     integral = constrain(integral, -1000, 1000);  // Integral wind-up protection
//     output += ki * integral;                  
//     output += kd * (error - lastError);       // Derivative term
//     lastError = error;

//     // Calculate motor speeds based on PID output
//     int leftMotorSpeed = motorSpeed + output;
//     int rightMotorSpeed = motorSpeed - output;

//     // Constrain motor speeds to the valid range
//     leftMotorSpeed = constrain(leftMotorSpeed, minSpeed, maxSpeed);
//     rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, maxSpeed);

//     // Debugging output
//     // Serial.print("Error: ");
//     // Serial.print(error);
//     // Serial.print(", Output: ");
//     // Serial.print(output);
//     // Serial.print(", Left Speed: ");
//     // Serial.print(leftMotorSpeed);
//     // Serial.print(", Right Speed: ");
//     // Serial.println(rightMotorSpeed);

//     // Set motor speeds
//     analogWrite(motorA_pin1, leftMotorSpeed);  // Left motor forward
//     analogWrite(motorA_pin2, 0);               // Left motor backward off
//     analogWrite(motorB_pin1, rightMotorSpeed); // Right motor forward
//     analogWrite(motorB_pin2, 0);               // Right motor backward off
// }


void line_following_pid_forward(char lineColor) {
    readIRValues();
    // Read sensor values


    // Calculate error value
    float error = 0.0;

    for (int i = 0; i < 8; i++) {
        if ((lineColor == 'W' && irValues[i+1] < threshold) || 
            (lineColor == 'B' && irValues[i+1] > threshold)) {
            error += (i - 3.275); // Center of sensor array is 3.275
        }
    }

    // PID control
    float output = kp * error;               // Proportional term
    integral += error;                       // Integral term
    output += ki * integral;                 // Integral term
    output += kd * (error - lastError);      // Derivative term
    lastError = error;

    // Adjust motor speeds based on PID output
    int leftMotorSpeed = motorSpeed + output;
    int rightMotorSpeed = motorSpeed - output;

    // Ensure motor speeds are within valid range
    leftMotorSpeed = constrain(leftMotorSpeed, minSpeed, maxSpeed);
    rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, maxSpeed);

    // Debugging outputs
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(", Output: ");
    Serial.print(output);
    Serial.print(", Left Speed: ");
    Serial.print(leftMotorSpeed);
    Serial.print(", Right Speed: ");
    Serial.println(rightMotorSpeed);

    // Set motor speeds
    analogWrite(motorA_pin1, leftMotorSpeed); // Left motor forward
    analogWrite(motorA_pin2, 0);             // Left motor backward disabled
    analogWrite(motorB_pin1, rightMotorSpeed); // Right motor forward
    analogWrite(motorB_pin2, 0);             // Right motor backward disabled
}

void runForDurationPID(unsigned long duration) {
    unsigned long startTime = millis(); // Record the start time
    while (millis() - startTime < duration) {
        line_following_pid_forward(color);  // Call your line following function
    }
}

void setup() {
    Serial.begin(115200);

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
      Serial.println(F("SSD1306 allocation failed"));   // no for loop if there is an error in display
      return;
    }

    // Initialize motor pins
    pinMode(motorA_pin1, OUTPUT);
    pinMode(motorA_pin2, OUTPUT);
    pinMode(motorB_pin1, OUTPUT);
    pinMode(motorB_pin2, OUTPUT);

    // Initialize IR sensor pins
    for (int i = 0; i < 10; i++) {
        pinMode(irPins[i], INPUT);
    }

      display.clearDisplay();     // Clear the display



      delay(100);
      displayText("Welcome");
      delay(2000);
      display.clearDisplay();
      
}


// motors

// #pragma region Motor Control
void turnLeft(int speed) {
    analogWrite(motorA_pin1, 0);
    analogWrite(motorA_pin2, speed);
    analogWrite(motorB_pin1, speed);
    analogWrite(motorB_pin2, 0);
}

void turnRight(int speed) {
    analogWrite(motorA_pin1, speed);
    analogWrite(motorA_pin2, 0);
    analogWrite(motorB_pin1, 0);
    analogWrite(motorB_pin2, speed);
}

void turnRightOneWheel(int speed) {
    analogWrite(motorA_pin1, speed);
    analogWrite(motorA_pin2, 0);
    analogWrite(motorB_pin1, 0);
    analogWrite(motorB_pin2, 0);
}


void turnLeftOneWheel(int speed) {
    analogWrite(motorA_pin1, 0);
    analogWrite(motorA_pin2, 0);
    analogWrite(motorB_pin1, speed);
    analogWrite(motorB_pin2, 0);
}


void stopMotors() {
    analogWrite(motorA_pin1, 0);
    analogWrite(motorA_pin2, 0);
    analogWrite(motorB_pin1, 0);
    analogWrite(motorB_pin2, 0);
}

void moveForward(int speed) {
    analogWrite(motorA_pin1, speed);
    analogWrite(motorA_pin2, 0);
    analogWrite(motorB_pin1, speed);
    analogWrite(motorB_pin2, 0);
}

// void right_rotation() {
//   const unsigned long initialRotateTime = 500;  // Initial small rotation duration
//   const unsigned long shortDelayTime = 10;      // Short delay between checks
//   unsigned long startTime = millis();

//   // Step 1: Perform initial small rotation
//   turnRightOneWheel(120);  // Start turning
//   while (millis() - startTime < initialRotateTime) {
//     // Wait for the initial rotation to complete (non-blocking)
//   }
//   stopMotors();  // Stop motors after initial rotation

//   // Step 2: Short delay for stabilization
//   startTime = millis();  // Reset start time
//   while (millis() - startTime < shortDelayTime) {
//     // Allow motors to stop briefly
//   }

//   // Step 3: Continue rotating until the middle IR detects the black line
//   while (!mid_IR_right(color)) {
//     turnRightOneWheel(100);  // Keep turning
//     readIRValues();          // Continuously read IR sensor
//     delay(5);                // Minimal delay for sensor stability
//   }

//   stopMotors();  // Stop once the line is detected

//   // Step 4: Final delay to ensure full stop (optional)
//   delay(1000);  // 1 second pause, can be adjusted or removed
// }


void right_rotation(){
 
  turnRightOneWheel(120); // 120
  delay(500); 
  stopMotors();
  delay(10);

  do{
    turnRightOneWheel(100); //120
    readIRValues();
    delay(5);
  }
  while(mid_IR_right(color)==false);
  stopMotors();
  delay(1000);


}


// void left_rotation() {
//   const unsigned long initialRotateTime = 200;  // Initial small rotation duration
//   const unsigned long shortDelayTime = 10;      // Short delay between checks
//   unsigned long startTime = millis();

//   // Step 1: Perform initial small rotation
//   turnLeft(100);  // Start turning
//   while (millis() - startTime < initialRotateTime) {
//     // Wait for the initial rotation to complete (non-blocking)
//   }
//   stopMotors();  // Stop motors after initial rotation

//   // Step 2: Short delay for stabilization
//   startTime = millis();  // Reset start time
//   while (millis() - startTime < shortDelayTime) {
//     // Allow motors to stop briefly
//   }

//   // Step 3: Continue rotating until the middle IR detects the black line
//   while (!mid_IR_left(color)) {
//     turnLeft(100);  // Keep turning
//     readIRValues();         // Continuously read IR sensor
//     delay(5);               // Minimal delay for sensor stability
//   }

//   stopMotors();  // Stop once the line is detected

//   // Step 4: Final delay to ensure full stop (optional)
//   delay(1000);  // 1 second pause, can be adjusted or removed
// }


void left_rotation(){

  turnLeft(100);
  delay(200);
  stopMotors();
  delay(5);

  do{
    turnLeft(100);
    readIRValues();
    delay(5);

  }
  // while(mid_IR_left(color)==false);
  while(mid_IR_leftx()==false && mid_IR_rightx()==false);
  stopMotors();
  delay(1000);


}


// void dead_end_rotation(){
//   turnLeft(100);
//   delay(200);
//   stopMotors();
//   delay(5);

//   do{
//     turnLeft(100);
//     readIRValues();
//     delay(5);

//   }
//   while(mid_IR_left(color)==false);
//   // while(mid_IR_leftx()==false && mid_IR_rightx()==false);
//   stopMotors();


// }
// #pragma endregion



//## IR

void  readIRValues() {
    for (int i = 0; i < 10; i++) {
        irValues[i] = analogRead(irPins[i]);
    }
}

bool mid_IR_rightx() {                //use for 'B' left rotate problem
  if (irValues[4] > threshold) {
    return true;
  }
    return false;
}

bool mid_IR_leftx() {
  if (irValues[3] > threshold) {
    return true;
  }
    return false;
}

bool mid_IR_right(char lineColor) {
  if ((lineColor == 'B' && (irValues[5] > threshold || irValues[6] > threshold)) || 
      (lineColor == 'W' && (irValues[4] < threshold || irValues[5] < threshold || irValues[6] < threshold ))) {
    return true;
  }
  return false;
}


bool mid_IR_left(char lineColor) {
  if ((lineColor == 'B' && (irValues[3] > threshold || irValues[4] > threshold)) || 
      (lineColor == 'W' && (irValues[3] < threshold || irValues[4] < threshold || irValues[5] < threshold))) {
    return true;
  }
  return false;
}

bool rightJunction(char lineColor) {
  if (lineColor == 'B') {
    // For black line: IR values should be greater than the threshold
    if ((irValues[9] > threshold || irValues[8] > threshold) &&
        (irValues[5] > threshold || irValues[4] > threshold)) {
      return true;
    }
  } else if (lineColor == 'W') {
    // For white line: IR values should be less than the threshold
    if ((irValues[9] < thresholdRight) &&
        (irValues[4] < threshold || irValues[5] > threshold)) {
      return true;
    }
  }
  return false;
}

bool leftJunction(char lineColor) {
  if (lineColor == 'B') {
    // For black line: compare if IR values are greater than the threshold
    if (irValues[1] > threshold && (irValues[5] > threshold || irValues[4] > threshold)) {
      return true;
    }
  } else if (lineColor == 'W') {
    // For white line: compare if IR values are less than the threshold
    if (irValues[0] < thresholdLeft && (irValues[5] < threshold || irValues[4] < threshold)) {
      return true;
    }
  }
  return false;
}

bool t_junction(char lineColor) {
  if (lineColor == 'B') {
    // For black line: IR values should be greater than the threshold
    // if ((irValues[0] > threshold || irValues[1] > threshold) &&
    //     (irValues[9] > threshold || irValues[8] > threshold) &&
    //     (irValues[5] > threshold || irValues[4] > threshold)) {
    //   return true;
    // }

    if ((irValues[0] > threshold || irValues[1] > threshold) &&
        (irValues[9] > threshold || irValues[8] > threshold ) &&
        (irValues[5] > threshold || irValues[4] > threshold)) {
      return true;
    }

  } else if (lineColor == 'W') {
    // For white line: IR values should be less than the threshold
    if ((irValues[0] < thresholdLeft || irValues[1] < threshold) &&
        (irValues[9] < thresholdRight || irValues[8] < thresholdRight) &&
        (irValues[5] < threshold || irValues[4] < threshold)) {
      return true;
    }
  }
  return false;
}


bool opposite_line(char lineColor) {
  if (lineColor == 'B') {
    if (irValues[0] < threshold && irValues[1] < threshold && irValues[2] < threshold && 
        irValues[3] < threshold && irValues[4] < threshold && irValues[5] < threshold && 
        irValues[6] < threshold && irValues[7] < threshold   && irValues[8] < threshold && irValues[9] < threshold  ) {
      return true;
    }
  } else if (lineColor == 'W') {
    if (irValues[0] > threshold && irValues[1] > threshold && irValues[2] > threshold && 
        irValues[3] > threshold && irValues[4] > threshold && irValues[5] > threshold && 
        irValues[6] > threshold && irValues[7] > threshold) {
      return true;
    }
  }
  return false;
}

bool Black_OR(char lineColor) {
  if (lineColor == 'B') {
    // For black line: IR values should be greater than the threshold
    if (irValues[0] > threshold || irValues[1] > threshold || irValues[2] > threshold || 
        irValues[3] > threshold || irValues[4] > threshold || irValues[5] > threshold || 
        irValues[6] > threshold || irValues[7] > threshold || irValues[8] > threshold || irValues[9] > threshold ) {
      return true;
    }
  } else if (lineColor == 'W') {
    // For white line: IR values should be less than the threshold
    if (irValues[0] < threshold || irValues[1] < threshold || irValues[2] < threshold || 
        irValues[3] < threshold || irValues[4] < threshold || irValues[5] < threshold || 
        irValues[6] < threshold || irValues[7] < threshold) {
      return true;
    }
  }
  return false;
}


bool color_changer(char lineColor) {
  if (lineColor == 'B') {
    // For black line: IR values should be greater or less than the threshold accordingly
    if (irValues[0] > thresholdRight && irValues[9] > thresholdLeft &&
        (irValues[3] < threshold || irValues[4] < threshold || irValues[5] < threshold || irValues[6] < threshold)) {
      return true;
    }
  } else if (lineColor == 'W') {
    // For white line: Reverse the comparisons
    if (irValues[0] < thresholdRight && irValues[9] < thresholdLeft &&
        (irValues[3] > threshold || irValues[4] > threshold  || irValues[5] > threshold || irValues[6] > threshold)) {
      return true;
    }
  }
  return false;
}







void loop() {

    //     displayText("We");
    // // line_following_pid_forward(color);

    readIRValues(); // Read all IR sensors
    if (t_junction(color)){
        displayText("T J");
        handleRightJunction();
    } 
    else if (rightJunction(color)) {
        displayText("Right J");
        handleRightJunction();
    } else if (leftJunction(color)) {
        displayText("Left J");
        handleLeftJunction();

    } else if (opposite_line(color)){
        displayText("Opposite");
        handleDeadJunction();
    } else if (color_changer(color)){
        displayText("Color change");
        toggleColor(color);
        stopMotors();
        delay(2000);
    } else {
        handleLineFollowing();
    }

    // delay(10); // Small delay for stability //30
}

void handleRightJunction() {
    // stopMotors();
    // delay(100);
    right_rotation();
    delay(10);
}

void handleLeftJunction() {
    stopMotors();
    delay(5);
    runForDurationPID(130);
    stopMotors();
    delay(10);
    readIRValues();

    if (opposite_line(color)) {
        left_rotation();
        delay(10);
        runForDurationPID(30);
    }else {
        handleLineFollowing();
    }
}

void handleDeadJunction(){
        stopMotors();
        // delay(20);
        runForDurationPID(40);
        stopMotors();
        // delay(20);
        readIRValues();

        if (Black_OR(color)){
          handleLineFollowing();
        }else{
          runForDurationPID(40);
          stopMotors();
          delay(20);
          readIRValues();

          if (Black_OR(color)){
              handleLineFollowing();
          } else{
            runForDurationPID(40);
            stopMotors();
            delay(20);
            readIRValues();

            if (Black_OR(color)){
              handleLineFollowing();
            }else{
              stopMotors();
              delay(1000);  
              left_rotation();         // Rotate 180 degrees
            }
            
          }
        }


}

void handleLineFollowing() {
    line_following_pid_forward(color);

}