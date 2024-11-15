#include "display.h"



void displayText(const String &text) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println(text);
    display.display();
}
