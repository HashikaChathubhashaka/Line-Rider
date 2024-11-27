#include <WiFi.h>
#include <ArduinoOTA.h>

const char* ssid = "Bytes";  // Your Wi-Fi SSID
const char* password = "hashika2000";  // Your Wi-Fi password

const int ledPin = 2;

void setup() {
  Serial.begin(115200);
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("Connected to Wi-Fi");
  
  pinMode(ledPin, OUTPUT);

  // Start OTA
  ArduinoOTA.setPassword("myOTAPassword");
  ArduinoOTA.begin();  
}

void loop() {
  ArduinoOTA.handle();  // Handle OTA requests
  

  digitalWrite(ledPin, HIGH); // Turn the LED on
  delay(200);                // Wait for 1 second (1000 ms)
  digitalWrite(ledPin, LOW);  // Turn the LED off
  delay(200);                // Wait for 1 second


}
