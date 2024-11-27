#ifndef IR_H
#define IR_H

const int irPins[10] = {39,14, 27, 26, 25, 33, 32, 35, 34,  36};
extern int irValues[10];
extern int thresholdsIR[10];

void readIRValues();
void autoCalibrateIR();
void loadThresholds();

bool right_junction(char lineColor);
bool left_junction(char lineColor);
bool mid_IR(char lineColor);    
bool dead_end_or_dotted(char lineColor);
bool all_oposite_color(char lineColor);




#endif // IR_H
