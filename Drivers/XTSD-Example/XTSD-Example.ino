
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "Wire.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!SD.begin()){
    Serial.println("SD Card Failed to Begin!");
    while(1);
  } else {
    Serial.println("SD Card Initialized!");
  }

  File file = SD.open("/Test-Telemetry.csv", FILE_WRITE);
  file.close();

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Beginning Main Loop...");
  delay(2000);

}

void loop() {
  // Read IMU Data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  String csv_line = String(a.acceleration.x) + "," + String(a.acceleration.y) + "," + String(a.acceleration.z);

  File file = SD.open("/Test-Telemetry.csv", FILE_APPEND);
  file.println(csv_line);

  Serial.print("File Size in Bytes: ");
  Serial.println(file.size());
  Serial.println(csv_line);

  file.close();

  
  delay(1000);

  // Write to SD

}
