#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// === GPS SETTINGS ===
#define GPS_RX 16  // ESP32 receives from GPS TX
#define GPS_TX 17  // ESP32 transmits to GPS RX
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);

// === RM3100 SETTINGS ===
#define RM3100_ADDR 0x20
#define REG_REVID  0x36
#define REG_POLL   0x00
#define REG_CMM    0x01
#define REG_STATUS 0x34
#define REG_CCX1   0x04
#define REG_CCX0   0x05
#define REG_MX     0x24
#define INITIAL_CC 200
float gain;

// === TIMING SETTINGS ===
unsigned long lastGPSPrint = 0;
const unsigned long gpsInterval = 1000;

unsigned long lastMagRead = 0;
const unsigned long magInterval = 200;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // SDA = 21, SCL = 22
  Wire.setClock(100000); // Optional: slower I2C for stability

  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // === RM3100 INIT ===
  Serial.println("Initializing RM3100...");
  uint8_t revid = readReg(REG_REVID);
  Serial.print("REVID = 0x"); Serial.println(revid, HEX);

  changeCycleCount(INITIAL_CC);
  uint16_t cc = (readReg(REG_CCX1) << 8) | readReg(REG_CCX0);
  gain = (0.3671 * cc) + 1.5;

  Serial.print("Cycle Count = "); Serial.println(cc);
  Serial.print("Gain = "); Serial.println(gain);

  writeReg(REG_CMM, 0x79); // Continuous mode
  Serial.println("Setup complete.\n");
}

void loop() {
  // === 1. Read one GPS byte (non-blocking) ===
  if (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }

  // === 2. Read and print magnetometer data every 200ms ===
  if (millis() - lastMagRead >= magInterval) {
    lastMagRead = millis();
    readMagnetometer();
  }

  // === 3. Print GPS info every 1000ms ===
  if (millis() - lastGPSPrint >= gpsInterval) {
    lastGPSPrint = millis();
    printGPSInfo();
  }
}

// === RM3100 FUNCTIONALITY ===

void readMagnetometer() {
  uint8_t status = readReg(REG_STATUS);
  if (!(status & 0x80)) {
    Serial.println("Magnetometer: Data not ready");
    return;
  }

  Wire.beginTransmission(RM3100_ADDR);
  Wire.write(REG_MX);
  Wire.endTransmission(false);
  Wire.requestFrom(RM3100_ADDR, 9);

  if (Wire.available() != 9) {
    Serial.println("Magnetometer: Incomplete read");
    return;
  }

  int32_t x = read24BitSigned();
  int32_t y = read24BitSigned();
  int32_t z = read24BitSigned();

  float xf = x / gain;
  float yf = y / gain;
  float zf = z / gain;
  float mag = sqrt(xf * xf + yf * yf + zf * zf);

  Serial.println("\n============ MAGNETOMETER INFO =============");
  Serial.print("X: "); Serial.print(xf);
  Serial.print("  Y: "); Serial.print(yf);
  Serial.print("  Z: "); Serial.print(zf);
  Serial.print("  | Magnitude: "); Serial.println(mag);
  Serial.println("============================================\n");
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

// === GPS FORMATTING ===
void printGPSInfo() {
  Serial.println("\n\n================== GPS INFO ==================");
  Serial.print("Latitude:       ");
  if (gps.location.isValid()) Serial.println(gps.location.lat(), 6);
  else Serial.println("Invalid");

  Serial.print("Longitude:      ");
  if (gps.location.isValid()) Serial.println(gps.location.lng(), 6);
  else Serial.println("Invalid");

  Serial.print("Altitude:       ");
  if (gps.altitude.isValid()) {
    Serial.print(gps.altitude.meters()); Serial.println(" m");
  } else Serial.println("Invalid");

  Serial.print("Speed (km/h):   ");
  if (gps.speed.isValid()) Serial.println(gps.speed.kmph());
  else Serial.println("Invalid");

  Serial.print("Speed (knots):  ");
  if (gps.speed.isValid()) Serial.println(gps.speed.knots());
  else Serial.println("Invalid");

  Serial.print("Bearing (COG):  ");
  if (gps.course.isValid()) {
    Serial.print(gps.course.deg()); Serial.println("°");
  } else Serial.println("Invalid");

  Serial.print("Satellites:     ");
  Serial.println(gps.satellites.value());

  Serial.print("HDOP:           ");
  if (gps.hdop.isValid()) Serial.println(gps.hdop.hdop());
  else Serial.println("Invalid");

  Serial.print("UTC Time:       ");
  if (gps.time.isValid()) {
    Serial.printf("%02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
  } else Serial.println("Invalid");

  Serial.print("UTC Date:       ");
  if (gps.date.isValid()) {
    Serial.printf("%02d/%02d/%04d\n", gps.date.day(), gps.date.month(), gps.date.year());
  } else Serial.println("Invalid");

  Serial.println("==============================================");
}