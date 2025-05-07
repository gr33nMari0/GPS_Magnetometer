#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <SD.h>

// === GPS SETTINGS ===
#define GPS_RX 16
#define GPS_TX 17
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

// === SD CARD SETTINGS ===
#define SD_CS_PIN 5
bool sdReady = false;
unsigned long logLineCount = 0;
String filename;

// === TIMING SETTINGS ===
unsigned long lastGPSPrint = 0;
const unsigned long gpsInterval = 1000;

unsigned long lastMagRead = 0;
const unsigned long magInterval = 200;

// === Magnetometer values ===
float magX = 0, magY = 0, magZ = 0, magMag = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // SDA = 21, SCL = 22
  Wire.setClock(100000);
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

  // === SD CARD INIT ===
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    sdReady = false;
  } else {
    Serial.println("SD card initialized.");
    sdReady = true;

    // Generate unique filename with timestamp or counter
    int fileIndex = 1;
    do {
      filename = "/log" + String(fileIndex++) + ".csv";
    } while (SD.exists(filename));

    File f = SD.open(filename, FILE_WRITE);
    if (f) {
      f.println("Line,MagX,MagY,MagZ,MagMagnitude,GPSLat,GPSLon,Altitude(m),Speed(km/h),Speed(knots),COG(deg),HDOP,Time(UTC),Date(UTC)");
      Serial.println("Header written to " + filename);
    }
    f.close();
  }
}

void loop() {
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }

  if (millis() - lastMagRead >= magInterval) {
    lastMagRead = millis();
    readMagnetometer();
  }

  if (millis() - lastGPSPrint >= gpsInterval) {
    lastGPSPrint = millis();
    printGPSInfo();
    writeDataToSD();
  }
}

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

  magX = x / gain;
  magY = y / gain;
  magZ = z / gain;
  magMag = sqrt(magX * magX + magY * magY + magZ * magZ);

  Serial.print("Mag X: "); Serial.print(magX);
  Serial.print(" Y: "); Serial.print(magY);
  Serial.print(" Z: "); Serial.print(magZ);
  Serial.print(" | Magnitude: "); Serial.println(magMag);
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
  for (int attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(RM3100_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) == 0) {
      Wire.requestFrom(RM3100_ADDR, 1);
      if (Wire.available()) return Wire.read();
    }
    delay(10);
  }
  Serial.printf("Failed to read reg 0x%02X after retries\n", reg);
  return 0;
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

void printGPSInfo() {
  Serial.println("\n\n================== GPS INFO ==================");
  Serial.print("Latitude:       ");
  Serial.println(gps.location.isValid() ? String(gps.location.lat(), 6) : "Invalid");

  Serial.print("Longitude:      ");
  Serial.println(gps.location.isValid() ? String(gps.location.lng(), 6) : "Invalid");

  Serial.print("Satellites:     ");
  Serial.println(gps.satellites.value());

  Serial.print("UTC Time:       ");
  if (gps.time.isValid()) {
    Serial.printf("%02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
  } else Serial.println("Invalid");

  Serial.println("==============================================");
}

void writeDataToSD() {
  if (!sdReady) {
    Serial.println("SD card not ready.");
    return;
  }

  File f = SD.open(filename, FILE_APPEND);
  if (f) {
    logLineCount++;

    f.print(logLineCount); f.print(",");
    f.print(magX); f.print(",");
    f.print(magY); f.print(",");
    f.print(magZ); f.print(",");
    f.print(magMag); f.print(",");

    if (gps.location.isValid() && gps.time.isValid() && gps.date.isValid()) {
      f.print(gps.location.lat(), 6); f.print(",");
      f.print(gps.location.lng(), 6); f.print(",");
      f.print(gps.altitude.meters()); f.print(",");
      f.print(gps.speed.kmph()); f.print(",");
      f.print(gps.speed.knots()); f.print(",");
      f.print(gps.course.deg()); f.print(",");
      f.print(gps.hdop.hdop()); f.print(",");
      
       // Format UTC time
      char timeBuffer[10];
      snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
      f.print(timeBuffer); f.print(",");

      // Format UTC date
      char dateBuffer[12];
      snprintf(dateBuffer, sizeof(dateBuffer), "%02d/%02d/%04d", gps.date.day(), gps.date.month(), gps.date.year());
      f.println(dateBuffer);
    } else {
      f.println("NaN,NaN,NaN,NaN,NaN,NaN,NaN");
    }

    f.close();
    Serial.print("Line "); Serial.print(logLineCount); Serial.println(" written to SD card.");
  } else {
    Serial.println("Error opening file for writing.");
  }
}

