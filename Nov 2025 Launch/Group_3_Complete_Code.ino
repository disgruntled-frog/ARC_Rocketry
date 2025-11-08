#include <Wire.h>

// BARO SECTION

#define BARO_ADDR         0x5C  //0x5D if SDA tied high

// Address of Registers
#define BARO_WHOAMI_REG   0x0F
#define BARO_CTRL_REG1    0x10     // ODR, LPF, SPI Mode
#define BARO_PRESS_OUT_XL 0x28   // LSB
#define BARO_PRESS_OUT_L  0x29   //
#define BARO_PRESS_OUT_H  0x2A   // MSB

#define BARO_WHOAMI_VAL 0xB3

const byte size = 1;
byte value[size] = {};

const byte baro_size = 3;
byte baro_data[baro_size] = {};

int32_t raw_baro = 0;
double baro = 0.0;

//IMU

#include <Wire.h>  

#define IMU_ADDR         0x6A  //0x6B if SDA tied high

// Address of Registers
#define IMU_WHOAMI_REG   0x0F

#define IMU_CTRL1        0x10     // ACC:  Op mode, ODR
#define IMU_CTRL2        0x11     // GYRO: Op mode, ODR
#define IMU_CTRL3        0x12     //  N/A   for now
#define IMU_CTRL4        0x13     //  N/A   for now
#define IMU_CTRL5        0x14     //  N/A   for now
#define IMU_CTRL6        0x15     // GYRO: LPF1, FS Sel
#define IMU_CTRL7        0x16     // GYRO: Enbl LPF`
#define IMU_CTRL8        0x17     // ACC:  LPF2, FS Sel
#define IMU_CTRL9        0x18     // ACC:  Configuring Filters
#define IMU_CTRL10       0x19     //  IMU:  Enbl dbg for Embedded functions and Self Testing

#define IMU_OUTX_L_G     0x22
#define IMU_OUTX_H_G     0x23
#define IMU_OUTY_L_G     0x24
#define IMU_OUTY_H_G     0x25
#define IMU_OUTZ_L_G     0x26
#define IMU_OUTZ_H_G     0x27
#define IMU_OUTX_L_A     0x28
#define IMU_OUTX_H_A     0x29
#define IMU_OUTY_L_A     0x2A
#define IMU_OUTY_H_A     0x2B
#define IMU_OUTZ_L_A     0x2C
#define IMU_OUTZ_H_A     0x2D

#define IMU_WHOAMI_VAL 0x70

////////
const int axis = 6;  // acc + gyro

const byte imu_size = axis*2;
byte imu_reg_data[axis*2] = {};

// This will be the parsed data
int16_t raw_imu[axis] = {};
double imu[axis] = {};
////////

// GPS / LORA SECTION

#include <Adafruit_GPS.h>
#include "SoftwareSerial.h"

#define GPSSerial Serial1
#define RX1_PIN 2 // GPS TX -> ESP32 RX2       DOUBLE CHECK ON ESP
#define TX1_PIN 4 // GPS RX -> ESP32 TX2

String lora_RX_address = "2";

Adafruit_GPS GPS(&GPSSerial);

HardwareSerial LoRaSerial(2);

SoftwareSerial lora(16, 17);
/* LORA RYLR998 Wiring

  TXD to ESP D4
  RXD to ESP D5

*/

#define GPSECHO false

uint32_t timer = 0;

// FUNCTION DECLARATIONS

byte readRegisters(byte _addr, byte _reg, byte* _value,byte _size = 1);
byte writeRegister(byte _addr, byte _reg, byte _value);

// baro
void config_baro();
void read_baro(byte* _value, int32_t& raw_baro, double& baro);

// imu
void config_imu();
void read_imu(byte* _value, int16_t* _raw_data, double* _data);

// Gps
float convertToDecimal(float nmeaValue);


void setup() {
 
  Serial.begin(9600);
  Wire.begin();

  Serial.println("Init Sensors Now...");

  config_baro();

  Serial.println("Configured, Starting Main Loop...");

 
  Serial.begin(9600);
  Wire.begin();

  Serial.println("Init Sensors Now...");

  config_imu();

  Serial.println("Configured, Starting Main Loop...");

  Serial.begin(115200); //Serial Baud Rate set to 115200
  Serial.println("ESP32 GPS Decimal Degrees Test");

 
  GPSSerial.begin(9600, SERIAL_8N1, RX1_PIN, TX1_PIN);
  lora.begin(9600);

 
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    
  GPS.sendCommand(PGCMD_ANTENNA);              

  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);           // request firmware version

}


void loop() {
 
    read_baro(baro_data, raw_baro, baro);
 
    // Comment line below to see on plotter
    Serial.print("Pressure in hPa: ");
    Serial.println(baro);
    delay(100);

    read_imu(imu_reg_data, raw_imu, imu);
 
    // Human Readable
   
    Serial.print("Gyro Data: ");
    Serial.print(imu[0]);
    Serial.print(", ");
    Serial.print(imu[1]);
    Serial.print(", ");
    Serial.print(imu[2]);
    Serial.print(" | Imu Data: ");
    Serial.print(imu[3]);
    Serial.print(", ");
    Serial.print(imu[4]);
    Serial.print(", ");
    Serial.println(imu[5]);
   

    // Serial Plotter Friendly
    /*
    Serial.print(imu[0]);
    Serial.print(", ");
    Serial.print(imu[1]);
    Serial.print(", ");
    Serial.print(imu[2]);
    Serial.print(", ");
    Serial.print(imu[3]);
    Serial.print(", ");
    Serial.print(imu[4]);
    Serial.print(", ");
    Serial.println(imu[5]);
    */

    delay(100);

   
  char c = GPS.read();
  if (GPSECHO && c) Serial.write(c);

 
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }

 
  if (millis() - timer > 1000) {
    timer = millis();
    Serial.println("----------------------");
    Serial.print("Fix: "); Serial.println(GPS.fix);
   
    float lat = convertToDecimal(GPS.latitude);
    if (GPS.lat == 'S') lat = -lat;

    float lon = convertToDecimal(GPS.longitude);
    if (GPS.lon == 'W') lon = -lon;

    Serial.print("Latitude: "); Serial.println(lat, 6);
    Serial.print("Longitude: "); Serial.println(lon, 6);
    Serial.print("Altitude (m): "); Serial.println(GPS.altitude);
    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);

    String msg = String(lat,6) + "," + String(lon,6);
    if (GPS.fix) {
      msg = "YG,"+ msg;
      Serial.println("YG added");
    }
     
    else {
      msg = "NG,"+ msg;
      Serial.println("NG added");
    }

    double length =msg.length();
   
    lora.println("AT+SEND=" + lora_RX_address + ","+ length +","+msg);
   
    Serial.println("msg sent");
    Serial.println("----------------------");    
  }

}

// Sensor

byte readRegisters(byte _addr, byte _reg, byte* _value,byte _size){
  // byte _addr: address of the I2C sensor you wish to communicate with
  // byte _reg: address of the register within said I2C sensor you wish to read from
  // byte* _value: pointer to the array you will store the value(s) read
  // byte _size: size of the value array
  // byte returnval: error code, 0 is normal operation

  Wire.beginTransmission(_addr);
  Wire.write(_reg);
  Wire.requestFrom(_addr, _size);
  while(Wire.available()){
    *_value = Wire.read();
    _value++;
  }
  return Wire.endTransmission();
}

byte writeRegister(byte _addr, byte _reg, byte _value){
  // byte _addr: address of the I2C sensor you wish to communicate with
  // byte _reg: address of the register within said I2C sensor you wish to write to
  // byte _value: byte you wish to write to said register

  Wire.beginTransmission(_addr);
  Wire.write(_reg);
  Wire.write(_value);
  return Wire.endTransmission();
}

void config_baro(){
  // This will be default configuration for everyone, future plans to add customization
  byte results = writeRegister(BARO_ADDR, BARO_CTRL_REG1, B01110000); // ODR 200 Hz, No Low pass, Coninuous Mode, 4-wire SPI
  (void)results;
}

void read_baro(byte* _value, int32_t& raw_baro, double& baro){
  readRegisters(BARO_ADDR,BARO_PRESS_OUT_XL,baro_data,baro_size);

 
  raw_baro = (baro_data[2]<<16)|(baro_data[1]<<8)|(baro_data[0]);  
  // Convert to hPa
  baro = raw_baro / 4096.0;
}

void config_imu(){
  // This will be default configuration for everyone, future plans to add customization
  byte results = writeRegister(IMU_ADDR, IMU_CTRL1, B00001000); // High-performance, ODR 480 Hz
  results = writeRegister(IMU_ADDR, IMU_CTRL2, B00001000); // High-performance, ODR 480 Hz
  results = writeRegister(IMU_ADDR, IMU_CTRL6, B00001100); // LPF1: 175, FS +/- 4000 dps
  results = writeRegister(IMU_ADDR, IMU_CTRL8, B00000111); // BW: ODR/4, FS +/- 32 g
  results = writeRegister(IMU_ADDR, IMU_CTRL9, B00001000); // Selecting output from ACC LPF2. Come back to this one to tune fitlering
  (void)results;
}

void read_imu(byte* _value, int16_t* _raw_data, double* _data){
  readRegisters(IMU_ADDR,IMU_OUTX_L_G,_value,imu_size);   //_value == imu_reg_data

  // Byte Concatenation: _raw_data -> [gyroX,gyroY,gyroZ,accX,accY,accZ]
  _raw_data[0] = (_value[1]<<8)|(_value[0]);
  _raw_data[1] = (_value[3]<<8)|(_value[2]);
  _raw_data[2] = (_value[5]<<8)|(_value[4]);

  _raw_data[3] = (_value[7]<<8)|(_value[6]);
  _raw_data[4] = (_value[9]<<8)|(_value[8]);
  _raw_data[5] = (_value[11]<<8)|(_value[10]);

  // Convert Gyro into dps
  double fs_dps = 4000;
  double max_16_signed = 32767;
  double gyro_scale_factor = fs_dps / max_16_signed;

  for (int i=0; i<3; i++){
    _data[i] = _raw_data[i] * gyro_scale_factor;
  }

 
  double fs_g = 32;
  double acc_scale_factor = fs_g / max_16_signed;

  for (int i=3; i<6; i++){
    _data[i] = _raw_data[i] * acc_scale_factor;
  }

}



float convertToDecimal(float nmeaValue) {
  int deg = int(nmeaValue / 100);        // degrees
  float min = nmeaValue - deg * 100;     // minutes
  return deg + min / 60.0;
}
