#include <Wire.h>
#include <Adafruit_GPS.h>
#include <HardwareSerial.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// ------------------ GPS ------------------
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
HardwareSerial GPSSerial(2);   // GPS on Serial2
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false

// ------------------ LoRa ------------------
HardwareSerial LoRaSerial(1);  // LoRa on Serial1
String lora_RX_address = "2";

// ------------------ BAROMETER (LPS22HH) ------------------
#define BARO_ADDR         0x5C
#define BARO_CTRL_REG1    0x10
#define BARO_PRESS_OUT_XL 0x28
#define BARO_PRESS_OUT_L  0x29
#define BARO_PRESS_OUT_H  0x2A
const byte baro_size = 3;
byte baro_data[baro_size] = {};
int32_t raw_baro = 0;
double baro = 0.0;

// ------------------ IMU (MPU6050) ------------------
#define IMU_ADDR         0x68
#define MPU_PWR_MGMT_1   0x6B
#define MPU_SMPLRT_DIV   0x19
#define MPU_CONFIG       0x1A
#define MPU_GYRO_CONFIG  0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_ACCEL_XOUT_H 0x3B
const int axis = 6; // gyroX,Y,Z + accX,Y,Z
const byte imu_size = axis*2;
byte imu_reg_data[imu_size] = {};
int16_t raw_imu[axis] = {};
double imu[axis] = {};

// ------------------ TIMERS ------------------
uint32_t timerSensors = 0;
uint32_t timerGPS = 0;

// ------------------ I2C READ/WRITE ------------------
byte readRegisters(byte _addr, byte _reg, byte* _value, byte _size = 1){
  Wire.beginTransmission(_addr);
  Wire.write(_reg);
  Wire.endTransmission(false);
  Wire.requestFrom(_addr, _size);
  byte i = 0;
  while (Wire.available() && i < _size) _value[i++] = Wire.read();
  return 0;
}

byte writeRegister(byte _addr, byte _reg, byte _value){
  Wire.beginTransmission(_addr);
  Wire.write(_reg);
  Wire.write(_value);
  return Wire.endTransmission();
}

// ------------------ BAROMETER ------------------
void config_baro() { writeRegister(BARO_ADDR, BARO_CTRL_REG1, B01110000); }

void read_baro(byte* _value, int32_t& raw_baro, double& baro){
  readRegisters(BARO_ADDR, BARO_PRESS_OUT_XL, baro_data, baro_size);
  raw_baro = (baro_data[2]<<16) | (baro_data[1]<<8) | baro_data[0];
  baro = raw_baro / 4096.0;
}

// ------------------ MPU6050 ------------------
void config_imu() {
  writeRegister(IMU_ADDR, MPU_PWR_MGMT_1, 0x00);
  writeRegister(IMU_ADDR, MPU_SMPLRT_DIV, 0x07);
  writeRegister(IMU_ADDR, MPU_CONFIG, 0x06);
  writeRegister(IMU_ADDR, MPU_GYRO_CONFIG, 0x18);
  writeRegister(IMU_ADDR, MPU_ACCEL_CONFIG, 0x10);
}

void read_imu(byte* _value, int16_t* _raw_data, double* _data){
  readRegisters(IMU_ADDR, MPU_ACCEL_XOUT_H, imu_reg_data, imu_size);
  // Gyro
  _raw_data[0] = (imu_reg_data[8]<<8)|imu_reg_data[9];
  _raw_data[1] = (imu_reg_data[10]<<8)|imu_reg_data[11];
  _raw_data[2] = (imu_reg_data[12]<<8)|imu_reg_data[13];
  // Acc
  _raw_data[3] = (imu_reg_data[0]<<8)|imu_reg_data[1];
  _raw_data[4] = (imu_reg_data[2]<<8)|imu_reg_data[3];
  _raw_data[5] = (imu_reg_data[4]<<8)|imu_reg_data[5];

  double gyro_scale = 2000.0 / 32768.0;
  for (int i=0;i<3;i++) _data[i] = _raw_data[i]*gyro_scale;

  double acc_scale = 8.0 / 32768.0;
  for (int i=3;i<6;i++) _data[i] = _raw_data[i]*acc_scale;
}

// ------------------ GPS UTILS ------------------
float convertToDecimal(float nmeaValue) {
  int deg = int(nmeaValue / 100);
  float min = nmeaValue - deg * 100;
  return deg + min / 60.0;
}

// ------------------ SD CARD ------------------
const int SD_CS_PIN = 5; // For Adafruit XTSD 512 MB
bool sdReady = false;

void initSD() {
  Serial.println("Initializing SPI Flash SD Card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card Mount Failed");
    sdReady = false;
    return;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    sdReady = false;
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) Serial.println("MMC");
  else if (cardType == CARD_SD) Serial.println("SDSC");
  else if (cardType == CARD_SDHC) Serial.println("SDHC");
  else Serial.println("UNKNOWN");

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %llu MB\n", cardSize);

  sdReady = true;

  // Test write/read
  File file = SD.open("/test.txt", FILE_WRITE);
  if (file) {
    file.println("Hello ESP32 SPI Flash SD!");
    file.close();
    Serial.println("File written successfully.");
  } else {
    Serial.println("Failed to open file for writing.");
  }

  file = SD.open("/test.txt");
  if (file) {
    Serial.println("Reading file:");
    while (file.available()) {
      Serial.write(file.read());
    }
    file.close();
    Serial.println();
  } else {
    Serial.println("Failed to open file for reading.");
  }
}

// ------------------ SETUP ------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // GPS
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);
  Serial.println("GPS Initialized. Waiting for fix...");

  // LoRa
  LoRaSerial.begin(9600, SERIAL_8N1, 26, 25);

  // Sensors
  config_baro();
  config_imu();
  Serial.println("Sensors initialized.");

  // SD card
  initSD();
}

// ------------------ LOOP ------------------
void loop() {
  // ---------------- Sensors ----------------
  if (millis() - timerSensors > 1000) {
    timerSensors = millis();
    Serial.println("---------------------------");
    read_baro(baro_data, raw_baro, baro);
    read_imu(imu_reg_data, raw_imu, imu);

    Serial.print("Gyro (deg/s): ");
    Serial.print(imu[0]); Serial.print(", "); Serial.print(imu[1]); Serial.print(", "); Serial.println(imu[2]);

    Serial.print("Accel (g): ");
    Serial.print(imu[3]); Serial.print(", "); Serial.print(imu[4]); Serial.print(", "); Serial.println(imu[5]);

    Serial.print("Pressure (hPa): "); Serial.println(baro);
    Serial.println("---------------------------");

    // --------- SD Card Logging ---------
    if (sdReady) {
        File logFile = SD.open("/sensor_log.csv", FILE_APPEND);
        if (logFile) {
            float lat = convertToDecimal(GPS.latitude);
            if (GPS.lat == 'S') lat = -lat;
            float lon = convertToDecimal(GPS.longitude);
            if (GPS.lon == 'W') lon = -lon;

            // Serial print for debug
            Serial.print("Logging to SD: ");
            Serial.print(millis()); Serial.print(",");
            Serial.print(imu[0]); Serial.print(",");
            Serial.print(imu[1]); Serial.print(",");
            Serial.print(imu[2]); Serial.print(",");
            Serial.print(imu[3]); Serial.print(",");
            Serial.print(imu[4]); Serial.print(",");
            Serial.print(imu[5]); Serial.print(",");
            Serial.print(baro); Serial.print(",");
            Serial.print(lat, 6); Serial.print(",");
            Serial.println(lon, 6);

            // Write to SD
            logFile.print(millis()); logFile.print(",");
            logFile.print(imu[0]); logFile.print(",");
            logFile.print(imu[1]); logFile.print(",");
            logFile.print(imu[2]); logFile.print(",");
            logFile.print(imu[3]); logFile.print(",");
            logFile.print(imu[4]); logFile.print(",");
            logFile.print(imu[5]); logFile.print(",");
            logFile.print(baro); logFile.print(",");
            logFile.print(lat, 6); logFile.print(",");
            logFile.println(lon, 6);

            logFile.close();
        } else {
            Serial.println("Failed to open sensor_log.csv for writing");
        }
    }
  }

  // ---------------- GPS ----------------
  char c = GPS.read();
  if (GPSECHO && c) Serial.write(c);

  if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());

  if (millis() - timerGPS > 1000) {
    timerGPS = millis();

    float lat = convertToDecimal(GPS.latitude);
    if (GPS.lat == 'S') lat = -lat;

    float lon = convertToDecimal(GPS.longitude);
    if (GPS.lon == 'W') lon = -lon;

    String msg;
    if (GPS.fix) {
      msg = "YG," + String(lat, 6) + "," + String(lon, 6);
      Serial.println("=== GPS Fix ===");
      Serial.print("Lat: "); Serial.println(lat, 6);
      Serial.print("Lon: "); Serial.println(lon, 6);
      Serial.print("Alt: "); Serial.println(GPS.altitude);
      Serial.print("Sats: "); Serial.println((int)GPS.satellites);
      Serial.println("================");
    } else {
      msg = "NG," + String(lat, 6) + "," + String(lon, 6);
      Serial.println("Waiting for GPS fix...");
    }

    // Send GPS over LoRa
    LoRaSerial.println("AT+SEND=" + lora_RX_address + "," + msg.length() + "," + msg);
    Serial.println("Message sent: " + msg);
  }
}
