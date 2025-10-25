
#include <Wire.h>

// Defining Address of Sensor
#define BARO_ADDR         0x5C  //0x5D if SDA tied high

// Defining Address of Registers
#define BARO_WHOAMI_REG   0x0F
#define BARO_CTRL_REG1    0x10     // ODR, LPF, SPI Mode
#define BARO_PRESS_OUT_XL 0x28   // LSB
#define BARO_PRESS_OUT_L  0x29   // 
#define BARO_PRESS_OUT_H  0x2A   // MSB



// Defining Values
#define BARO_WHOAMI_VAL 0xB3



const byte size = 1;
byte value[size] = {};

const byte baro_size = 3;
byte baro_data[baro_size] = {};

int32_t raw_baro = 0;
double baro = 0.0;

// Function Declaration
byte readRegisters(byte _addr, byte _reg, byte* _value,byte _size = 1);
byte writeRegister(byte _addr, byte _reg, byte _value);
void config_baro();
void read_baro(byte* _value, int32_t& raw_baro, double& baro);


void setup() {
  
  Serial.begin(9600);
  Wire.begin();

  Serial.println("Init Sensors Now...");

  config_baro();

  Serial.println("Configured, Starting Main Loop...");

}

void loop() {
  
    read_baro(baro_data, raw_baro, baro);
  
    // Comment line below to see on plotter
    Serial.print("Pressure in hPa: ");
    Serial.println(baro);
    delay(100);

}


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
}

void read_baro(byte* _value, int32_t& raw_baro, double& baro){
  readRegisters(BARO_ADDR,BARO_PRESS_OUT_XL,baro_data,baro_size);

  // Byte Concatenation
  raw_baro = (baro_data[2]<<16)|(baro_data[1]<<8)|(baro_data[0]);  // baro_data == [LSB,midSB,MSB]
  // Convert to hPa
  baro = raw_baro / 4096.0;


}

