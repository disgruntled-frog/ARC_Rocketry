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


// Defining Objects
HardwareSerial gps_serial(1);  // The '1' indicates UART1
HardwareSerial lora_serial(2); // The '2' indicates UART2



// Defining Pins
const int RX_PIN_1 = ; 
const int TX_PIN_1 = ;


// Defining Addresses (if applicable for drivers)



// Defining Global Variables
unsigned long prev_Micros = micros()    // Used to time the loop (so we can get timestamps on data csv)
const long loop_Dur = 2000              // 500 Hz, can change based on how fast we can (reliably) get I2C on these protoboards




// Function Declaration
void init_gps()
void init_lora()
void init_sd()
float convertToDecimal(float nmeaValue)



void setup() {
  // Initializing comm buses
  Serial.begin(115200); //Serial Baud Rate set to 115200
  mySerial1.begin(9600, SERIAL_8N1, RX_PIN_1, TX_PIN_1);

  Wire.begin()


  // Initializing Sensors


  // Set up SD Card
}

void loop() {
  // Read Acc

  // Read Gyro

  // Read Baro

  // If 

}




void init_gps(){
  // Initialize Serial1 on custom pins
  GPSSerial.begin(9600, SERIAL_8N1, RX1_PIN, TX1_PIN);


  // Configure GPS output and update rate
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // RMC + GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // 1 Hz
  GPS.sendCommand(PGCMD_ANTENNA);               // antenna status

  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);           // request firmware version
}

void init_lora(){
  lora.begin(9600);

  // can add more to this later
}

float convertToDecimal(float nmeaValue) {
  int deg = int(nmeaValue / 100);        // degrees
  float min = nmeaValue - deg * 100;     // minutes
  return deg + min / 60.0;
}