#include <Arduino.h>
#include <Wire.h>

// RM3100 I²C address (P2 and P4 pulled LOW)
#define RM3100_ADDR 0x20

// DRDY pin (optional)
#define PIN_DRDY 18  // Change if wired differently

// RM3100 internal register addresses
#define REG_REVID     0x36
#define REG_POLL      0x00
#define REG_CMM       0x01
#define REG_STATUS    0x34
#define REG_CCX1      0x04
#define REG_CCX0      0x05
#define REG_MX        0x24  // Measurement register starting at MX2

// Options
#define INITIAL_CC     200  // Cycle count
#define USE_SINGLE     0    // 0 = continuous mode, 1 = single measurement
#define USE_DRDY       1    // 1 = use DRDY pin, 0 = use status register

float gain;

void setup() {
  Serial.begin(115200);
  Wire.begin();  // Default: SDA = 21, SCL = 22 on ESP32
  pinMode(PIN_DRDY, INPUT);

  delay(100);

  uint8_t revid = readReg(REG_REVID);
  Serial.print("REVID = 0x");
  Serial.println(revid, HEX);  // Should be 0x22

  changeCycleCount(INITIAL_CC);

  uint16_t cc = (readReg(REG_CCX1) << 8) | readReg(REG_CCX0);
  gain = (0.3671 * cc) + 1.5;

  Serial.print("Cycle Count = ");
  Serial.println(cc);
  Serial.print("Gain = ");
  Serial.println(gain);

  if (USE_SINGLE) {
    writeReg(REG_CMM, 0x00);      // idle
    writeReg(REG_POLL, 0x70);     // trigger X, Y, Z
  } else {
    writeReg(REG_CMM, 0x79);      // continuous mode, no alarms
  }
}

void loop() {
  if (USE_DRDY) {
    while (digitalRead(PIN_DRDY) == LOW);
  } else {
    while (!(readReg(REG_STATUS) & 0x80));
  }

  // Request 9 bytes from MX2 to MZ0
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

  delay(200);
}

int32_t read24BitSigned() {
  uint8_t b2 = Wire.read();
  uint8_t b1 = Wire.read();
  uint8_t b0 = Wire.read();

  int32_t value = ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | b0;
  if (b2 & 0x80) value |= 0xFF000000;  // sign extension
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
  for (int i = 0; i < 3; i++) {  // X, Y, Z
    Wire.write(msb);
    Wire.write(lsb);
  }
  Wire.endTransmission();
}