#include <QMC5883LCompass.h>

QMC5883LCompass compass;

void setup() {
  Serial.begin(115200);
  compass.init();  // Initialize the QMC5883L

  // Optional: Adjust the calibration based on your environment
  compass.setCalibration(-1547, 994, -1121, 1343, -1929, 1723);
}

void loop() {
  int x, y, z;
  float heading;

  // Read compass values
  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();


  heading = compass.getAzimuth();

  Serial.print("X: "); Serial.print(x);
  Serial.print(" Y: "); Serial.print(y);
  Serial.print(" Z: "); Serial.print(z);
  Serial.print(" Heading: "); Serial.print(heading);
  Serial.println("°");

  delay(500);
}
