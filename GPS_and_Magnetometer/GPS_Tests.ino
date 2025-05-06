// Author: Noah Mikov

#include <HardwareSerial.h>

HardwareSerial SerialGPS(2);  // UART2 for GPS

void setup() {
  Serial.begin(115200);                     // USB Serial
  SerialGPS.begin(9600, SERIAL_8N1, 17, 16); // GPS RX=17, TX=16
  Serial.println("Listening to GPS...");
}

void loop() {
  while (SerialGPS.available()) {
    char c = SerialGPS.read();
    Serial.write(c);  // Echo raw NMEA sentence to USB monitor
  }
}
