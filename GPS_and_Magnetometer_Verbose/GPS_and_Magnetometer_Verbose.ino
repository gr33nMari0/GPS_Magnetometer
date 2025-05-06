#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// GPS UART Pins
#define GPS_RX 16
#define GPS_TX 17

// RM3100 I2C and control
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

TinyGPSPlus gps;
HardwareSerial SerialGPS(2); // UART2 for GPS
float gain;

// Track last GPS and magnetometer output time
unsigned long lastOutputTime = 0;
unsigned long outputInterval = 1000; // ms

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(PIN_DRDY, INPUT);
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Serial.println("=== Initializing RM3100 Magnetometer ===");

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

  Serial.println("=== Starting Sensor Loop ===\n");
}

void loop() {
  // Read and parse GPS characters
  while (SerialGPS.available()) {
    char c = SerialGPS.read();
    gps.encode(c);
    Serial.write(c); // echo raw NMEA for verbose logging
  }

  // Print GPS and Magnetometer info periodically
  if (millis() - lastOutputTime >= outputInterval) {
    lastOutputTime = millis();

    // === GPS DATA ===
    Serial.println("\n\n================== GPS INFO ==================");

    Serial.print("Latitude:       ");
    if (gps.location.isValid()) Serial.println(gps.location.lat(), 6);
    else Serial.println("Invalid");

    Serial.print("Longitude:      ");
    if (gps.location.isValid()) Serial.println(gps.location.lng(), 6);
    else Serial.println("Invalid");

    Serial.print("Altitude:       ");
    if (gps.altitude.isValid()) {
      Serial.print(gps.altitude.meters());
      Serial.println(" m");
    } else Serial.println("Invalid");

    Serial.print("Speed (km/h):   ");
    if (gps.speed.isValid()) Serial.println(gps.speed.kmph());
    else Serial.println("Invalid");

    Serial.print("Speed (knots):  ");
    if (gps.speed.isValid()) Serial.println(gps.speed.knots());
    else Serial.println("Invalid");

    Serial.print("Bearing (COG):  ");
    if (gps.course.isValid()) {
      Serial.print(gps.course.deg());
      Serial.println("°");
    } else Serial.println("Invalid");

    Serial.print("Satellites:     ");
    Serial.println(gps.satellites.value());

    Serial.print("HDOP:           ");
    if (gps.hdop.isValid()) Serial.println(gps.hdop.hdop());
    else Serial.println("Invalid");

    Serial.print("UTC Time:       ");
    if (gps.time.isValid()) {
      Serial.printf("%02d:%02d:%02d\n",
        gps.time.hour(),
        gps.time.minute(),
        gps.time.second());
    } else Serial.println("Invalid");

    Serial.print("UTC Date:       ");
    if (gps.date.isValid()) {
      Serial.printf("%02d/%02d/%04d\n",
        gps.date.day(),
        gps.date.month(),
        gps.date.year());
    } else Serial.println("Invalid");

    Serial.println("==============================================");

    // === Magnetometer ===
    bool ready = false;
    unsigned long startWait = millis();

    if (USE_DRDY) {
      while (millis() - startWait < 100) {
        if (digitalRead(PIN_DRDY) == HIGH) {
          ready = true;
          break;
        }
      }
    } else {
      while (millis() - startWait < 100) {
        if (readReg(REG_STATUS) & 0x80) {
          ready = true;
          break;
        }
      }
    }

    if (ready) {
      Wire.beginTransmission(RM3100_ADDR);
      Wire.write(REG_MX);
      Wire.endTransmission(false);
      Wire.requestFrom(RM3100_ADDR, 9);

      if (Wire.available() == 9) {
        int32_t x = read24BitSigned();
        int32_t y = read24BitSigned();
        int32_t z = read24BitSigned();

        float xf = x / gain;
        float yf = y / gain;
        float zf = z / gain;
        float mag = sqrt(xf * xf + yf * yf + zf * zf);

        Serial.println("\n============ MAGNETOMETER INFO =============");
        Serial.print("X: ");
        Serial.print(xf);
        Serial.print("  Y: ");
        Serial.print(yf);
        Serial.print("  Z: ");
        Serial.print(zf);
        Serial.print("  | Magnitude: ");
        Serial.println(mag);
        Serial.println("============================================\n");
      } else {
        Serial.println("Failed to read full magnetometer data");
      }
    } else {
      Serial.println("Magnetometer timeout or DRDY not ready");
    }
  }
}

// === RM3100 Functions ===
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