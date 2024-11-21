#include "display.h"
#include "ir.h"


void displayText(const String &text) {
    display.clearDisplay();
    display.setRotation(2);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(5, 10);
    display.println(text);
    display.display();
}


// make a function to display IR values like digital(compared to thier thesholds)
void displayWhiteBlack() {
    display.clearDisplay();
    display.setRotation(2);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(5, 10);

    for (int i = 0; i < 10; ++i) {
        if (irValues[i] > thresholdsIR[i]) {
            display.print("W "); // White
        } else {
            display.print("B "); // Black
        }
    }

    display.display();
}