


/*
  Organization: ARCUIA

  Date:         10-20-25

  Purpose:      Boilerplate code intended to be used as a starting point 
                  for your very own flight controllers

  Notes:        Do not edit GPS or LoRa code there are many variables we
                  need to account for. This code is critical for finding
                  your rockets
                
*/

// Imports
#include <Arduino.h>
#include <Wire.h>
#include "FS.h" // For File System functions
#include "SD.h" 
#include "SPI.h"
#include <Adafruit_GPS.h>
#include "SoftwareSerial.h"
#include <HardwareSerial.h>

// Sensor Libraries (Local Library + MPU6050 Library)
#include <LPS22HH.h>
#include <Adafruit_MPU6050.h>


// Defining Objects
HardwareSerial gps_serial(1);  // The '1' indicates UART1
HardwareSerial lora_serial(2); // The '2' indicates UART2
File telemetryFile;
String filename;


// Defining Pins
const int RX_PIN_1 = 26; 
const int TX_PIN_1 = 25;
const int RX_PIN_2 = 16; 
const int TX_PIN_2 = 17;


// Defining Addresses (if not using defaults for the library)
String lora_RX_address = "2";


// Defining Global Variables
unsigned long prev_Micros = micros();    // Used to time the loop (so we can get timestamps on data csv)
const long loop_Dur = 2000;              // 500 Hz, can change based on how fast we can (reliably) get I2C on these protoboards


// Create Sensor Objects 
LPS22HH baro;
Adafruit_MPU6050 mpu;
Adafruit_GPS GPS(&gps_serial);


// Value Arrays
// void read(int32_t& raw_baro, double& baro);
// void LSM6DSV32X::read(int16_t* _raw_data, double* _data)
int32_t raw_baro = 0;
double baro_data = 0.0;
int16_t raw_imu[6] = {};
double imu_data[6] = {};

sensors_event_t a, g, temp;



// Function Declaration
void init_gps();
void init_lora();
void init_sd();
void write_sd();
float convertToDecimal(float nmeaValue);
void send_gps(bool debug = true);
String getNextFilename(const String &base, const String &ext);



void setup() {
  // Initializing comm buses

  // UART 0/1/2
  Serial.begin(115200); //Serial Baud Rate set to 115200
  gps_serial.begin(9600, SERIAL_8N1, RX_PIN_1, TX_PIN_1);
  lora_serial.begin(9600, SERIAL_8N1, RX_PIN_2, TX_PIN_2);

  // GPS
  init_gps();

  // I2C 
  Wire.begin();

  // Initializing Sensors (Default Addresses)
  baro.config();

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Set up SD Card
  init_sd();
}

void loop() {
  unsigned long currentMicros = micros();

  // Check if enough time has passed since the last execution
  if (currentMicros - prev_Micros >= loop_Dur) {
    // Update prev
    prev_Micros = currentMicros;
    
    // Read Imu
    mpu.getEvent(&a, &g, &temp);

    // Janky shit so I dont need to rewrite everything
    raw_imu[0] = a.acceleration.x;
    raw_imu[1] = a.acceleration.y;
    raw_imu[2] = a.acceleration.z;
    raw_imu[3] = g.gyro.x;
    raw_imu[4] = g.gyro.y;
    raw_imu[5] = g.gyro.z;

    // Read Baro
    baro.read(raw_baro,baro_data);

    // If GPS Data, Send OTA and/or print to Serial
    send_gps();

    // Log data
    write_sd();

    /*
    Serial.print(imu_data[0]);
    Serial.print(",");
    Serial.print(imu_data[1]);
    Serial.print(",");
    Serial.print(imu_data[2]);
    Serial.print(",");
    Serial.print(imu_data[3]);
    Serial.print(",");
    Serial.print(imu_data[4]);
    Serial.print(",");
    Serial.print(imu_data[5]);
    Serial.print(",");
    Serial.println(baro_data);
`   */
  }

}



/****************************************************************************************/


void init_gps(){
  // Configure GPS output and update rate
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // RMC + GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // 1 Hz
  GPS.sendCommand(PGCMD_ANTENNA);               // antenna status

  delay(1000);
  gps_serial.println(PMTK_Q_RELEASE);           // request firmware version
}


void init_lora(){

  // can add more to this later
}


float convertToDecimal(float nmeaValue) {
  int deg = int(nmeaValue / 100);        // degrees
  float min = nmeaValue - deg * 100;     // minutes
  return deg + min / 60.0;
}


void init_sd(){
  if (!SD.begin()) {
    Serial.println("SD card initialization failed!");
    while (true);
  }
  Serial.println("SD card initialized.");

  filename = getNextFilename("/telemetry_", ".csv");
  telemetryFile = SD.open(filename, FILE_WRITE);

  if (telemetryFile) {
    Serial.print("Created file: ");
    Serial.println(filename);
    telemetryFile.println("IMU,,,,,,BARO"); // Formatting, add in positions later
    telemetryFile.close();
  } else {
    Serial.println("Error creating file!");
  }
}

void write_sd(){

  telemetryFile = SD.open(filename, FILE_APPEND);

  String data_string = "";

  for (int i = 0; i < 6; i++) {
    data_string += String(raw_imu[i]); 
    if (i < 5) {
      data_string += ", "; 
    }
  }
  data_string += String(raw_baro);

  telemetryFile.println(data_string);
  
}

void send_gps(bool debug){

    if (debug){
      Serial.print("Fix: "); Serial.println(GPS.fix); //Prints if fixed (aka if connected to SAT)
    }

    float lat = convertToDecimal(GPS.latitude);
    if (GPS.lat == 'S') lat = -lat; //South = -lattitude

    float lon = convertToDecimal(GPS.longitude);
    if (GPS.lon == 'W') lon = -lon; //West = -longitude

    if (debug){
      Serial.print("Latitude: "); Serial.println(lat, 6); //Prints to Serial for debugging
      Serial.print("Longitude: "); Serial.println(lon, 6);
      Serial.print("Altitude (m): "); Serial.println(GPS.altitude);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }

    String msg = String(lat,6) + "," + String(lon,6); //String(lat/lon, # of decimal Points(rounds up) )
    if (GPS.fix) {
      msg = "YG,"+ msg; //adds YG to message = Yes GPS Connected
      if (debug){
        Serial.println("YG added");
      }
    }
    else {
      msg = "NG,"+ msg; //adds NG to message = No GPS Connected
      if (debug){
        Serial.println("NG added");
      }
    }

    double length =msg.length(); 
    // To transmit requires: Reciever Address, message length, and message.
    lora_serial.println("AT+SEND=" + lora_RX_address + ","+ length +","+msg); 
    //AT+SEND is the transmit command for the RYLR
    if (debug){
      Serial.println("msg sent");
      Serial.println("----------------------");
    }
}

// Returns the next unused file name like /telemetry_1.csv, /telemetry_2.csv, etc.
String getNextFilename(const String &base, const String &ext) {
  int fileIndex = 1;
  String path;
  do {
    path = base + String(fileIndex) + ext;
    fileIndex++;
  } while (SD.exists(path));
  return path;
}

