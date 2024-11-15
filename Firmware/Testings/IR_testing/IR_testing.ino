// left to right 
int irPins[10] = {14, 27, 26, 25, 33, 32, 35, 34, 39, 36}; 
int irValues[10];

void setup() {
    Serial.begin(115200);

    for (int i = 0; i < 10; i++) {
        pinMode(irPins[i], INPUT);
    }

}

void loop() {
  for (int i = 0; i < 10; i++) {
    irValues[i] = analogRead(irPins[i]);
  }

  Serial.print("IR Sensor Values: ");
  for (int i = 0; i < 10; i++) {
    Serial.print("IR");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(irValues[i]);
    if (i < 9) Serial.print(", ");
  }
  Serial.println();
  delay(500);
}
