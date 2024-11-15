#define BUTTON_1 15  


void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_1, INPUT_PULLUP);  // Set the pin as input with internal pull-up resistor


}

void loop() {
  // Read the button state (LOW when pressed, HIGH when not pressed)
  int buttonState1 = digitalRead(BUTTON_1);

  if (buttonState1 == LOW) {
    Serial.println("Button 1 Pressed");
  } else {
    Serial.println("Button 1 Not Pressed");
  }




  delay(200);  // Delay for readability
}
