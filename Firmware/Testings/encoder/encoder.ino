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

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

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

  Serial.println("Encoder test started. Rotate the motors to see encoder counts.");
}

void loop() {
  // Read the encoder counts (no interrupts blocked)
  long leftCount, rightCount;
  noInterrupts();
  leftCount = leftEncoderCount;
  rightCount = rightEncoderCount;
  interrupts();

  // Display the counts on the Serial Monitor
  Serial.print("Left Encoder Count: ");
  Serial.print(leftCount);
  Serial.print("\tRight Encoder Count: ");
  Serial.println(rightCount);

  delay(100); // Small delay for readability
}
