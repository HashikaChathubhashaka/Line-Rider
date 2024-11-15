#include <WiFi.h>
#include <ArduinoOTA.h>

const char* ssid = "Bytes";  // Your Wi-Fi SSID
const char* password = "Hashika2000";  // Your Wi-Fi password

void setup() {
  Serial.begin(115200);
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("Connected to Wi-Fi");

  // Start OTA
  ArduinoOTA.begin();  
}

void loop() {
  ArduinoOTA.handle();  // Handle OTA requests
  
  // Blink the LED as an example (initial state)
  digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED on
  delay(1000);                      // Wait for 1 second
  digitalWrite(LED_BUILTIN, LOW);   // Turn the LED off
  delay(1000);                      // Wait for 1 second
}
