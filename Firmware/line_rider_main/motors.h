#ifndef MOTORS_H
#define MOTORS_H

//left ->  A
const int motorA_pin1 = 16;  
const int motorA_pin2 = 4;

//right -> B
const int motorB_pin1 = 18;
const int motorB_pin2 = 17;

// Function Declaration for moveForward
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void stopMotors();
void testmotors();
void turnRightOneWheel(int speed);
void turnLeftOneWheel(int speed);
#endif // MOTORS_H
