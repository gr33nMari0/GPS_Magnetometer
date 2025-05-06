// Author: Noah Mikov

#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>

// RM3100 I²C address
#define RM3100_ADDR 0x20
#define PIN_DRDY 18

#define REG_REVID     0x36
#define REG_POLL      0x00
#define REG_CMM       0x01
#define REG_STATUS    0x34
#define REG_CCX1      0x04
#define REG_CCX0      0x05
#define REG_MX        0x24

#define INITIAL_CC     200
#define USE_SINGLE     0
#define USE_DRDY       1

// GPS setup
#define GPS_RX_PIN 16  // ESP32 receives on this pin (connect to GPS TX)
#define GPS_TX_PIN 17  // ESP32 transmits on this pin (connect to GPS RX)

TinyGPSPlus gps;
HardwareSerial SerialGPS(2);  // Use UART2 (Serial2)

float gain;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(PIN_DRDY, INPUT);

  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);  // Start GPS

  delay(100);

  uint8_t revid = readReg(REG_REVID);
  Serial.print("REVID = 0x");
  Serial.println(revid, HEX);

  changeCycleCount(INITIAL_CC);

  uint16_t cc = (readReg(REG_CCX1) << 8) | readReg(REG_CCX0);
  gain = (0.3671 * cc) + 1.5;

  Serial.print("Cycle Count = ");
  Serial.println(cc);
  Serial.print("Gain = ");
  Serial.println(gain);

  if (USE_SINGLE) {
    writeReg(REG_CMM, 0x00);
    writeReg(REG_POLL, 0x70);
  } else {
    writeReg(REG_CMM, 0x79);
  }
}

void loop() {
  // --- Read GPS data
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }

  // --- Wait for magnetometer data
  if (USE_DRDY) {
    while (digitalRead(PIN_DRDY) == LOW);
  } else {
    while (!(readReg(REG_STATUS) & 0x80));
  }

  // Read 9 bytes of magnetic data
  Wire.beginTransmission(RM3100_ADDR);
  Wire.write(REG_MX);
  Wire.endTransmission(false);
  Wire.requestFrom(RM3100_ADDR, 9);
  if (Wire.available() != 9) return;

  int32_t x = read24BitSigned();
  int32_t y = read24BitSigned();
  int32_t z = read24BitSigned();

  float xf = x / gain;
  float yf = y / gain;
  float zf = z / gain;
  float mag = sqrt(xf * xf + yf * yf + zf * zf);

  Serial.print("Magnetic Field [uT] - X: ");
  Serial.print(xf);
  Serial.print(" Y: ");
  Serial.print(yf);
  Serial.print(" Z: ");
  Serial.print(zf);
  Serial.print(" | Magnitude: ");
  Serial.println(mag);

  // --- Print GPS if valid
  if (gps.location.isUpdated()) {
    Serial.print("GPS Location: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("GPS Location: Not yet fixed");
  }

  delay(200);
}

int32_t read24BitSigned() {
  uint8_t b2 = Wire.read();
  uint8_t b1 = Wire.read();
  uint8_t b0 = Wire.read();
  int32_t value = ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | b0;
  if (b2 & 0x80) value |= 0xFF000000;
  return value;
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(RM3100_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(RM3100_ADDR, 1);
  return Wire.available() ? Wire.read() : 0;
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(RM3100_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void changeCycleCount(uint16_t cc) {
  uint8_t msb = (cc >> 8) & 0xFF;
  uint8_t lsb = cc & 0xFF;
  Wire.beginTransmission(RM3100_ADDR);
  Wire.write(REG_CCX1);
  for (int i = 0; i < 3; i++) {
    Wire.write(msb);
    Wire.write(lsb);
  }
  Wire.endTransmission();
}