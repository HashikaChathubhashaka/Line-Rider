#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>


Preferences preferences;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels


#define BUTTON_1 15  
#define BUTTON_2 0


int main_section = 1; // 0 button fucntions , 1 run

//  For button Functionalities 
int mode = 0;  // 0-sleep , 1-calibraion , 2-line following
int mode_on =0; // 0 and 1 

// for section of line following 
int section =1; // 0 for starting , 1 for running

char color = 'W';

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
const int thresholdRight = 200; 
const int thresholdLeft = 200;

// // B" PID constants
const float kpB = 25.0; // Proportional gain
const float kiB = 0.0; // Integral gain
const float kdB = 2.0; // Derivative gain

// W" PID constants
const float kpW = 4.0; // Proportional gain - 8 
const float kiW = 0.0; // Integral gain
const float kdW = 0.5; // Derivative gain

// PID variables
float integral = 0.0;
float lastErrorW = 0.0;
float lastErrorB = 0.0;

// Motor speed settings
const int motorSpeed = 100; // Base speed 120
const int maxSpeed = 200;   // Maximum speed 255
const int minSpeed = 30;     // Minimum speed 30

int irValues[10];



void toggleColor(char &color) {
  if (color == 'B') {
    color = 'W';  // Change from Black ('B') to White ('W')
  } else if (color == 'W') {
    color = 'B';  // Change from White ('W') to Black ('B')
  }
}




// calibration 
int thresholdsIR[10] = {200,200,200,200,200,200,200,200,200,200}; // Threshold values for IR sensors

int manualIR[10] = {200,200,200,200,200,200,200,200,200,200};



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
    for (int t = 0; t < 900; t++) { // Run for 200 cycles (~adjust as needed)
        readIRValues();
        for (int i = 0; i < 10; i++) {
            int sensorValue = irValues[i];
            if (sensorValue < minValues[i]) minValues[i] = sensorValue;
            if (sensorValue > maxValues[i]) maxValues[i] = sensorValue;
        }

        // // Move robot left and right alternately
        if (t < 450) {
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






void displayText(const String &text) {
    display.clearDisplay();
    display.setRotation(2);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(5, 10);
    display.println(text);
    display.display();
}


void line_following_pid_forward(char lineColor) {

  if (lineColor == 'W'){

    readIRValues();  // Function to read IR sensor values into irValues array

    // Define weights for each sensor, assuming 7 sensors are used
    int weights[10] = {0 ,5 , 4, 2.5, 1, -1, -2.5, -4, -5 ,0};  // Adjust as needed

    // Calculate the error value using weighted sensor readings
    float error = 0.0;
    // int activeSensors = 0;  // Track the number of sensors detecting the line

    for (int i = 0; i < 10; i++) {
        bool onLine = irValues[i] > threshold;

        if (onLine) {
            error += weights[i];  // Accumulate error using sensor weights
            // activeSensors++;
        }
    }

    // PID Control Calculation
    float output = kpW * error;                // Proportional term
    integral += error;                        // Integral term
    integral = constrain(integral, -1000, 1000);  // Integral wind-up protection
    output += kiW * integral;                  
    output += kdW * (error - lastErrorW);       // Derivative term
    lastErrorW = error;

    // Calculate motor speeds based on PID output
    int leftMotorSpeed = motorSpeed + output;
    int rightMotorSpeed = motorSpeed - output;

    // Constrain motor speeds to the valid range
    leftMotorSpeed = constrain(leftMotorSpeed, minSpeed, maxSpeed);
    rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, maxSpeed);

    // Debugging output
    // Serial.print("Error: ");
    // Serial.print(error);
    // Serial.print(", Output: ");
    // Serial.print(output);
    // Serial.print(", Left Speed: ");
    // Serial.print(leftMotorSpeed);
    // Serial.print(", Right Speed: ");
    // Serial.println(rightMotorSpeed);

    // Set motor speeds
    analogWrite(motorA_pin1, leftMotorSpeed);  // Left motor forward
    analogWrite(motorA_pin2, 0);               // Left motor backward off
    analogWrite(motorB_pin1, rightMotorSpeed); // Right motor forward
    analogWrite(motorB_pin2, 0);               // Right motor backward off


  }
  else{

    readIRValues();
    // Read sensor values


    // Calculate error value
    float error = 0.0;

    for (int i = 0; i < 8; i++) {
        if (irValues[i+1] > threshold) {
            error += (i - 3.275); // Center of sensor array is 3.275
        }
    }

    // PID control
    float output = kpB * error;               // Proportional term
    integral += error;                       // Integral term
    output += kiB * integral;                 // Integral term
    output += kdB * (error - lastErrorB);      // Derivative term
    lastErrorB = error;

    // Adjust motor speeds based on PID output
    int leftMotorSpeed = motorSpeed + output;
    int rightMotorSpeed = motorSpeed - output;

    // Ensure motor speeds are within valid range
    leftMotorSpeed = constrain(leftMotorSpeed, minSpeed, maxSpeed);
    rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, maxSpeed);

    // Debugging outputs
    // Serial.print("Error: ");
    // Serial.print(error);
    // Serial.print(", Output: ");
    // Serial.print(output);
    // Serial.print(", Left Speed: ");
    // Serial.print(leftMotorSpeed);
    // Serial.print(", Right Speed: ");
    // Serial.println(rightMotorSpeed);

    // Set motor speeds
    analogWrite(motorA_pin1, leftMotorSpeed); // Left motor forward
    analogWrite(motorA_pin2, 0);             // Left motor backward disabled
    analogWrite(motorB_pin1, rightMotorSpeed); // Right motor forward
    analogWrite(motorB_pin2, 0);             // Right motor backward disabled

  }
}





// void line_following_pid_forward(char lineColor) {
//     readIRValues();
//     // Read sensor values


//     // Calculate error value
//     float error = 0.0;

//     for (int i = 0; i < 8; i++) {
//         if ((lineColor == 'W' && irValues[i+1] < threshold) || 
//             (lineColor == 'B' && irValues[i+1] > threshold)) {
//             error += (i - 3.275); // Center of sensor array is 3.275
//         }
//     }

//     // PID control
//     float output = kp * error;               // Proportional term
//     integral += error;                       // Integral term
//     output += ki * integral;                 // Integral term
//     output += kd * (error - lastError);      // Derivative term
//     lastError = error;

//     // Adjust motor speeds based on PID output
//     int leftMotorSpeed = motorSpeed + output;
//     int rightMotorSpeed = motorSpeed - output;

//     // Ensure motor speeds are within valid range
//     leftMotorSpeed = constrain(leftMotorSpeed, minSpeed, maxSpeed);
//     rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, maxSpeed);

//     // Debugging outputs
//     Serial.print("Error: ");
//     Serial.print(error);
//     Serial.print(", Output: ");
//     Serial.print(output);
//     Serial.print(", Left Speed: ");
//     Serial.print(leftMotorSpeed);
//     Serial.print(", Right Speed: ");
//     Serial.println(rightMotorSpeed);

//     // Set motor speeds
//     analogWrite(motorA_pin1, leftMotorSpeed); // Left motor forward
//     analogWrite(motorA_pin2, 0);             // Left motor backward disabled
//     analogWrite(motorB_pin1, rightMotorSpeed); // Right motor forward
//     analogWrite(motorB_pin2, 0);             // Right motor backward disabled
// }

void runForDurationPID(unsigned long duration) {
    unsigned long startTime = millis(); // Record the start time
    while (millis() - startTime < duration) {
        Serial.println(millis() - startTime); // Debugging output
        line_following_pid_forward(color);    // Call your line following function
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

    // loadThresholds();
      delay(1000);
      moveForward(100);
      delay(400);
      stopMotors();
      delay(1000);



      
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
//   const unsigned long initialRotateTime = 200;  // Initial small rotation duration
//   const unsigned long shortDelayTime = 10;      // Short delay between checks
//   unsigned long startTime = millis();

//   // Step 1: Perform initial small rotation
//   turnRight(120);  // Start turning
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
//     turnRight(100);  // Keep turning
//     readIRValues();          // Continuously read IR sensor
//     delay(5);                // Minimal delay for sensor stability
//   }

//   stopMotors();  // Stop once the line is detected

//   // Step 4: Final delay to ensure full stop (optional)
//   delay(1000);  // 1 second pause, can be adjusted or removed
// }


// void right_rotation(){
 
//   turnRightOneWheel(120); // 120
//   delay(500); 
//   stopMotors();
//   delay(10);

//   do{
//     turnRightOneWheel(100); //120
//     readIRValues();
//     delay(5);
//   }
//   while(mid_IR_right(color)==false);
//   stopMotors();
//   delay(1000);


// }


void right_rotation(){          //two weel
 
  turnRight(120);
  delay(200);

  stopMotors();
  delay(5);

  do{
    turnRight(100);
    readIRValues();
    delay(5);

  }
  while(mid_IR_right(color)==false);
  stopMotors();
  delay(50);


}


// void left_rotation() {
//   const unsigned long initialRotateTime = 500;  // Initial small rotation duration
//   const unsigned long shortDelayTime = 10;      // Short delay between checks
//   unsigned long startTime = millis();

//   // Step 1: Perform initial small rotation
//   turnLeftOneWheel(120);  // Start turning
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
//     turnLeftOneWheel(100);  // Keep turning
//     readIRValues();         // Continuously read IR sensor
//     delay(5);               // Minimal delay for sensor stability
//   }

//   stopMotors();  // Stop once the line is detected

//   // Step 4: Final delay to ensure full stop (optional)
//   delay(1000);  // 1 second pause, can be adjusted or removed
// }


void left_rotation(){

  turnLeftOneWheel(120); // 120
  delay(350); 
  stopMotors();
  delay(1000);

  do{
    turnLeftOneWheel(120); //120
    readIRValues();
    delay(10);
  }
  while(mid_IR_left(color)==false);
  // while(mid_IR_leftx()==false && mid_IR_rightx()==false);
  stopMotors();
  delay(50);



}

bool previous_rotate_dir = true ; // 0 left

void dead_end_rotation(bool dir){
  if (dir){
    turnLeft(100);
    delay(200);

    stopMotors();
    delay(5);

    do{
      turnLeft(100);
      readIRValues();
      delay(5);

    }
    while(mid_IR_left(color)==false);
    // while(mid_IR_leftx()==false && mid_IR_rightx()==false);
    stopMotors();
    delay(100);

  }

  else{
    turnRight(100);
    delay(200);

    stopMotors();
    delay(5);

    do{
      turnRight(100);
      readIRValues();
      delay(5);

    }
    while(mid_IR_right(color)==false);
    stopMotors();
    delay(1000);

  }

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
  if (irValues[4] > thresholdsIR[4]) {
    return true;
  }
    return false;
}

bool mid_IR_leftx() {
  if (irValues[3] > thresholdsIR[3] ) {
    return true;
  }
    return false;
}

bool mid_IR_right(char lineColor) {
  if ((lineColor == 'B' && (irValues[4] < thresholdsIR[4] || irValues[5] > thresholdsIR[5] || irValues[6] > thresholdsIR[6])) || 
      (lineColor == 'W' && (irValues[4] < thresholdsIR[4] || irValues[5] < thresholdsIR[5] || irValues[3] < thresholdsIR[3]  ))) {
    return true;
  }
  return false;
}


bool mid_IR_left(char lineColor) {
  if ((lineColor == 'B' && (irValues[3] > thresholdsIR[3] || irValues[4] > thresholdsIR[4] || irValues[5] < thresholdsIR[5] )) || 
      (lineColor == 'W' && (irValues[6] < thresholdsIR[6] || irValues[4] < thresholdsIR[4] || irValues[5] < thresholdsIR[5]  ))) {
    return true;
  }
  return false;
}

bool rightJunction(char lineColor) {
  if (lineColor == 'B') {
    // For black line: IR values should be greater than the threshold
    if ((irValues[9] > thresholdsIR[9] || irValues[8] > thresholdsIR[8]) &&
        (irValues[5] > thresholdsIR[5] || irValues[4] > thresholdsIR[4])) {
      return true;
    }
  } else if (lineColor == 'W') {
    // For white line: IR values should be less than the threshold
    if ((irValues[9] < thresholdsIR[9]) &&
        (irValues[4] < thresholdsIR[4] || irValues[5] > thresholdsIR[5])) {
      return true;
    }
  }
  return false;
}

bool leftJunction(char lineColor) {
  if (lineColor == 'B') {
    // For black line: compare if IR values are greater than the threshold
    if (irValues[0] > thresholdLeft && irValues[1] > thresholdsIR[1]   && (irValues[5] > thresholdsIR[5] || irValues[4] > thresholdsIR[4])) {
      return true;
    }
  } else if (lineColor == 'W') {
    // For white line: compare if IR values are less than the threshold
    if (irValues[0] < thresholdLeft  && irValues[1] < thresholdsIR[1]  && (irValues[5] < thresholdsIR[5] || irValues[4] < thresholdsIR[4])) {
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

    if ((irValues[0] > thresholdLeft || irValues[1] > thresholdsIR[1]) &&
        (irValues[9] > thresholdsIR[9] || irValues[8] > thresholdsIR[8] ) &&
        (irValues[5] > thresholdsIR[5] || irValues[4] > thresholdsIR[4])) {
      return true;
    }

  } else if (lineColor == 'W') {
    // For white line: IR values should be less than the threshold
    if ((irValues[0] < thresholdLeft || irValues[1] < thresholdsIR[1]) &&
        (irValues[9] < thresholdsIR[9] || irValues[8] < thresholdsIR[9]) &&
        (irValues[5] < thresholdsIR[5] || irValues[4] < thresholdsIR[4])) {
      return true;
    }
  }
  return false;
}


bool opposite_line(char lineColor) {
  if (lineColor == 'B') {
    if (irValues[0] < thresholdLeft && irValues[1] < thresholdsIR[1] && irValues[2] < thresholdsIR[2] && 
        irValues[3] < thresholdsIR[3] && irValues[4] < thresholdsIR[4] && irValues[5] < thresholdsIR[5] && 
        irValues[6] < thresholdsIR[6] && irValues[7] < thresholdsIR[7]   && irValues[8] < thresholdsIR[8] && irValues[9] < thresholdsIR[9]  ) {
      return true;
    }
  } else if (lineColor == 'W') {
    if (irValues[0] >thresholdLeft && irValues[1] > thresholdsIR[1] && irValues[2] > thresholdsIR[2] && 
        irValues[3] > thresholdsIR[3] && irValues[4] > thresholdsIR[4] && irValues[5] > thresholdsIR[5] && 
        irValues[6] > thresholdsIR[6] && irValues[7] > thresholdsIR[7]) {
      return true;
    }
  }
  return false;
}

bool Black_OR(char lineColor) {
  if (lineColor == 'B') {
    // For black line: IR values should be greater than the threshold
    if (irValues[0] > thresholdLeft || irValues[1] > thresholdsIR[1] || irValues[2] > thresholdsIR[2] || 
        irValues[3] > thresholdsIR[3] || irValues[4] > thresholdsIR[4] || irValues[5] > thresholdsIR[5] || 
        irValues[6] > thresholdsIR[6] || irValues[7] > thresholdsIR[7] || irValues[8] > thresholdsIR[8] || irValues[9] > thresholdsIR[9] ) {
      return true;
    }
  } else if (lineColor == 'W') {
    // For white line: IR values should be less than the threshold
    if (irValues[0] < thresholdLeft || irValues[1] < thresholdsIR[1] || irValues[2] < thresholdsIR[2] || 
        irValues[3] < thresholdsIR[3] || irValues[4] < thresholdsIR[4] || irValues[5] < thresholdsIR[5] || 
        irValues[6] < thresholdsIR[6] || irValues[7] < thresholdsIR[7]) {
      return true;
    }
  }
  return false;
}


bool color_changer(char lineColor) {
  if (lineColor == 'B') {
    // For black line: IR values should be greater or less than the threshold accordingly
    if (irValues[0] > thresholdLeft && irValues[9] > thresholdsIR[9] &&
        (irValues[3] < thresholdsIR[3] || irValues[4] < thresholdsIR[4] || irValues[5] < thresholdsIR[5] || irValues[6] < thresholdsIR[6])) {
      return true;
    }
  } else if (lineColor == 'W') {
    // For white line: Reverse the comparisons
    if (irValues[0] < thresholdLeft && irValues[9] < thresholdsIR[9] &&
        (irValues[3] > thresholdsIR[3] || irValues[4] > thresholdsIR[4]  || irValues[5] > thresholdsIR[5] || irValues[6] > thresholdsIR[6])) {
      return true;
    }
  }
  return false;
}







void loop() {

  // For button Functionalities
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
        displayText("threshold mode");

        //serial monitor shoe threshold values
        for (int i = 0; i < 10; i++) {
            Serial.print(thresholdsIR[i]);
            Serial.print(" ");
        }
            Serial.println("");

        
        if (mode_on == 1) {
          displayText("changed threshold");
          delay(1000);
          loadThresholds();
          mode_on=0;
        }

  
    } 
    else if (mode == 1) {
        if (mode_on == 0) {
            displayText("Calibration mode");
        } else if (mode_on == 1) {
            displayText("Calibrating");
            delay(1000);
            autoCalibrateIR();
            //code for calibration

            displayText("Calibrate done");
            delay(2000);
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

  // For line following mode
  else if (main_section == 1) {
    if (section == 0) {
      //readIRValues(); 

      moveForward(100);
      delay(400);
      stopMotors();
      delay(1000);
      section=1;

      // if (leftJunction(color) || rightJunction(color) )
      // {
      //   stopMotors();
      //   displayText("Start"); 
      //   delay(10);
      //   runForDurationPID(400);
      //   stopMotors();


      //   section=1;
      // }

      // else{
      //   line_following_pid_forward(color);

      // }

    }

    else if(section == 1){


    // int buttonState1 = digitalRead(BUTTON_1); // Read the state of the first button

    // if (buttonState1 == LOW) {
    //     stopMotors();
    //     main_section = 0;
    // }


    readIRValues(); // Read all IR sensors

    if (color_changer(color)){
        displayText("Color change");
        toggleColor(color);
        stopMotors();
        delay(2000);

    }


    else if (leftJunction(color)) {

        displayText("Left J");
        previous_rotate_dir = true;
        handleLeftJunction();

    } else if (rightJunction(color)) {
        displayText("Right J");
        previous_rotate_dir=false;
        handleRightJunction();
    } else if (opposite_line(color)){

        displayText("Opposite");
        handleDeadJunction();
        previous_rotate_dir = !previous_rotate_dir;


    } 
    //else if (color_changer(color)){
    //     displayText("Color change");
    //     toggleColor(color);
    //     stopMotors();
    //     delay(2000);

    // } 
    else {
        handleLineFollowing();
    }


    }

  }




}




void handleRightJunction() {
    stopMotors();
    delay(10);
    runForDurationPID(130);
    // moveForward(100);
    // delay(180);
    stopMotors();
    delay(10);
    readIRValues();

    if (!Black_OR(color) ) {
        // delay(1000);
        right_rotation();
        // delay(10);
        // runForDurationPID(30);
    }else {
        handleLineFollowing();
    }
}

// void handleRightJunction() {
//     // stopMotors();
//     // delay(100);
//     right_rotation();
//     delay(10);
// }

// void handleLeftJunction() {
//     stopMotors();
//     delay(5);
//     runForDurationPID(130);
//     stopMotors();
//     delay(10);
//     readIRValues();

//     if (opposite_line(color)) {
//         left_rotation();
//         delay(10);
//         runForDurationPID(30);
//     }else {
//         handleLineFollowing();
//     }
// }


void handleLeftJunction() {
    stopMotors();
    delay(5);
    left_rotation();
    delay(10);

}

void handleDeadJunction(){
        stopMotors();
        delay(10);
        runForDurationPID(40);
        stopMotors();
        delay(10);
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
              dead_end_rotation(previous_rotate_dir);         // Rotate 180 degrees
            }
            
          }
        }


}

void handleLineFollowing() {
    line_following_pid_forward(color);

}