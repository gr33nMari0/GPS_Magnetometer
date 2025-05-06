#include <Wire.h>
#include <Adafruit_RM3100.h>
Adafruit_RM3100 mag;  // Create an RM3100 object

void setup() {
  Serial.begin(57600); // connect serial
  Serial.println("STARTING MAGNETOMETER");
  
  // Initialize the RM3100 magnetometer
  if (!mag.begin()) {
    Serial.println("Could not find a valid RM3100 sensor, check wiring!");
    while (1);
  }
  Serial.println("RM3100 sensor initialized.");
}
void loop() {
  // Get GPS data
  while (Serial1.available()) {
    if (gps.encode(Serial1.read())) { // encode gps data
      gps.f_get_position(&lon, &lat); // get latitude and longitude
      Serial.print("Longitude: ");
      Serial.print(lon, 8);
      Serial.print(",");
      Serial.print("Latitude: ");
      Serial.print(lat, 8);
      Serial.print(",");

      // Read RM3100 magnetometer data
      float magX, magY, magZ;
      mag.readMagnetometer(&magX, &magY, &magZ);  // Read magnetometer data

      // Print magnetic field values
      Serial.print("Magnetometer X: ");
      Serial.print(magX);
      Serial.print(", Y: ");
      Serial.print(magY);
      Serial.print(", Z: ");
      Serial.println(magZ);

      // Use the magnetometer values for further processing (e.g., heading calculation)
      getMagneticField(magX, magY, magZ);

      // Calculate positions, save to SD, and update OLED display
      gpsCalculate();
      saveSD();
      showOLED();
    }
  }
}

void getMagneticField(float magX, float magY, float magZ) {
  // Use the RM3100 data to calculate the magnetic field strength or heading
  // Example of calculating the magnitude of the magnetic field
  float magStrength = sqrt(magX * magX + magY * magY + magZ * magZ);
  Serial.print("Magnetic Field Strength: ");
  Serial.println(magStrength);
}
void saveSD() {
  myFile = SD.open(fileName, FILE_WRITE);
  Serial.println("Writing data...");
  
  // Write GPS and magnetometer data to SD card
  myFile.print(lon, 9);
  myFile.print(",");
  myFile.print(lat, 9);
  myFile.print(",");
  
  // Write magnetic field values (X, Y, Z) or field strength
  myFile.print(magX, 1);
  myFile.print(",");
  myFile.print(magY, 1);
  myFile.print(",");
  myFile.println(magZ, 1);

  myFile.close();
}
void showOLED() {
  display.clearDisplay();

  // GPS
  display.setCursor(4, 3);
  display.setTextColor(BLACK);
  display.setTextSize(2);
  display.fillCircle(9, 9, 9, WHITE);
  display.println("G");

  // Display RM3100 magnetometer data
  display.setCursor(50, 2);
  display.setTextSize(1);
  display.print("MagX: ");
  display.println(magX, 1);
  display.setCursor(50, 14);
  display.print("MagY: ");
  display.println(magY, 1);
  display.setCursor(50, 26);
  display.print("MagZ: ");
  display.println(magZ, 1);

  // SD-card
  display.fillRect(3, 50, 8, 17, WHITE);
  display.fillRect(11, 55, 5, 12, WHITE);
  display.fillTriangle(11, 50, 15, 54, 12, 54, WHITE);
  display.setCursor(4, 54);
  display.setTextColor(BLACK);
  display.setTextSize(1);
  display.println("SD");

  // Display the heading
  display.setCursor(2, 33);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.println(heading, 0);
  display.drawCircle(24, 34, 1, WHITE);

  display.display();
}
