#include <QMC5883LCompass.h>

QMC5883LCompass compass;

// Motor control pins
const int MOTOR_LEFT_FORWARD_PIN = 16;
const int MOTOR_LEFT_BACKWARD_PIN = 4;
const int MOTOR_RIGHT_FORWARD_PIN = 18;
const int MOTOR_RIGHT_BACKWARD_PIN = 17;

// Robot constants
const int TURN_SPEED = 100;    // Speed for rotation
const int TARGET_MARGIN = 3;  // Acceptable margin of error in degrees

void setup() {
  Serial.begin(115200);

  // Initialize compass
  compass.init();

  Serial.println("Compass initialized!");

  // Motor pin setup
  pinMode(MOTOR_LEFT_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD_PIN, OUTPUT);
}

void loop() {
  Serial.println("Enter target angle (0-360 degrees): ");
  while (Serial.available() == 0) {
    delay(100);
  }

  int targetAngle = Serial.parseInt(); // Get the target angle
  targetAngle = targetAngle % 360;    // Keep the angle within 0-360 degrees
  Serial.print("Rotating to: ");
  Serial.println(targetAngle);

  rotateToAngle(targetAngle);

  Serial.println("Rotation complete.");
  delay(2000);  // Wait before allowing a new input
}

void rotateToAngle(int targetAngle) {
  int currentHeading = getHeading();

  while (true) {
    currentHeading = getHeading();

    // Calculate the shortest rotation direction
    int error = targetAngle - currentHeading;
    if (error > 180) {
      error -= 360;
    } else if (error < -180) {
      error += 360;
    }

    // Check if the robot is within the margin of the target
    if (abs(error) <= TARGET_MARGIN) {
      stopMotors();
      break;
    }

    // Rotate based on the direction of error
    if (error > 0) {
      rotateRight(TURN_SPEED);
    } else {
      rotateLeft(TURN_SPEED);
    }

    Serial.print("Current Heading: ");
    Serial.print(currentHeading);
    Serial.print(" | Target: ");
    Serial.print(targetAngle);
    Serial.print(" | Error: ");
    Serial.println(error);

    delay(100);  // Small delay for stability
  }
}

int getHeading() {
  compass.read();
  int heading = compass.getAzimuth(); // Get the azimuth (heading) in degrees
  return heading;
}

void rotateRight(int speed) {
  analogWrite(MOTOR_LEFT_FORWARD_PIN, speed);
  analogWrite(MOTOR_LEFT_BACKWARD_PIN, 0);
  analogWrite(MOTOR_RIGHT_FORWARD_PIN, 0);
  analogWrite(MOTOR_RIGHT_BACKWARD_PIN, speed);
}

void rotateLeft(int speed) {
  analogWrite(MOTOR_LEFT_FORWARD_PIN, 0);
  analogWrite(MOTOR_LEFT_BACKWARD_PIN, speed);
  analogWrite(MOTOR_RIGHT_FORWARD_PIN, speed);
  analogWrite(MOTOR_RIGHT_BACKWARD_PIN, 0);
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_FORWARD_PIN, 0);
  analogWrite(MOTOR_LEFT_BACKWARD_PIN, 0);
  analogWrite(MOTOR_RIGHT_FORWARD_PIN, 0);
  analogWrite(MOTOR_RIGHT_BACKWARD_PIN, 0);
}
