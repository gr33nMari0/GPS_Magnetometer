#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

#define RX 16  // GPS TX → ESP32 RX
#define TX 17  // GPS RX ← ESP32 TX

TinyGPSPlus gps;
HardwareSerial SerialGPS(2);  // UART2 for GPS

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, RX, TX);
  Serial.println("Reading GPS data...");
}

void loop() {
  while (SerialGPS.available()) {
    char c = SerialGPS.read();
    gps.encode(c);

    // Echo raw GPS characters
    Serial.write(c);

    if (gps.location.isUpdated()) {
      Serial.println("\n\n================== GPS INFO ==================");

      Serial.print("Latitude:       ");
      Serial.println(gps.location.lat(), 6);

      Serial.print("Longitude:      ");
      Serial.println(gps.location.lng(), 6);

      Serial.print("Altitude:       ");
      Serial.print(gps.altitude.meters());
      Serial.println(" m");

      Serial.print("Speed (km/h):   ");
      Serial.print(gps.speed.kmph());
      Serial.println(" km/h");

      Serial.print("Speed (knots):  ");
      Serial.print(gps.speed.knots());
      Serial.println(" knots");

      Serial.print("Bearing (COG):  ");
      if (gps.course.isValid()) {
        Serial.print(gps.course.deg());
        Serial.println("°");
      } else {
        Serial.println("Not valid");
      }

      Serial.print("Satellites:     ");
      Serial.println(gps.satellites.value());

      Serial.print("HDOP:           ");
      Serial.println(gps.hdop.hdop());

      Serial.print("UTC Time:       ");
      if (gps.time.isValid()) {
        Serial.printf("%02d:%02d:%02d\n",
          gps.time.hour(),
          gps.time.minute(),
          gps.time.second());
      } else {
        Serial.println("Not valid");
      }

      Serial.print("UTC Date:       ");
      if (gps.date.isValid()) {
        Serial.printf("%02d/%02d/%04d\n",
          gps.date.day(),
          gps.date.month(),
          gps.date.year());
      } else {
        Serial.println("Not valid");
      }

      Serial.println("==============================================\n\n");

      delay(1000);  // wait between prints
    }
  }
}