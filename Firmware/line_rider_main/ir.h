#ifndef IR_H
#define IR_H

const int irPins[10] = {39,14, 27, 26, 25, 33, 32, 35, 34,  36};
extern int irValues[10];
extern int thresholdsIR[10];

void readIRValues();
void autoCalibrateIR();
void loadThresholds();



#endif // IR_H
