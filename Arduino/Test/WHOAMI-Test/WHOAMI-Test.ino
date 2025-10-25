/*
  Author:       Sam Manley
  Date:         10-25-25
  Purpose:      Generalized Test Code for Sensors
  Description:
                Use this sketch to do a simple WHOAMI test, this will confirm that:
                  - I2C bus is working fine (Will return an error != 0 if there is)
                  - Sensor is connected to I2C correctly (It wont acknowledge the I2C commands, either with NACK or something else)
                What it will NOT be able to tell you:
                  - Anything Sig Integ related, this is a pass/fail test, youll likely need a more complex setup to check for such things
                  - If your driver/library code configures the sensor correct, that is on you guys to verify (see me if questions)
  

  Notes:
                - If you simply want to find all I2C addresses on the bus, use I2C scanner that will live in the same directory as this on Github
                - If your sensor is not on here, feel free to add in a define block and commit to github
                - Make sure to only have one define block uncommented at a time to ensure no conflicting defines

*/

#include <Wire.h>


// UNCOMMENT ONLY YOUR SENSOR TO TEST
#define LPS22HH
// #define LSM6DSV32X
// #define LSM6DSV320X
// #define LSM6DSV80X
// #define LIS3MDL

  


#ifdef LPS22HH
  #define ADDR       0x5C  //0x5D if SDA tied high
  #define WHOAMI_REG 0x0F
  #define WHOAMI_VAL 0xB3
#endif

#ifdef LSM6DSV32X
  #define ADDR       0x5C  //0x5D if SDA tied high
  #define WHOAMI_REG 0x0F
  #define WHOAMI_VAL 0xB3
#endif


const byte size = 1;
byte value[size] = {};

const byte mag_size = 6;
byte mag_data[mag_size] = {};

// Function Declaration
byte readRegisters(byte _addr, byte _reg, byte* _value,byte _size = 1);
byte writeRegister(byte _addr, byte _reg, byte _value);


void setup() {
  
  Serial.begin(9600);
  Wire.begin();

  Serial.println("Init Sensors Now...");

}

void loop() {
  
  // Note: result is NOT what you recieve from the sensor, it is an error code from the I2C Bus
  byte result;
  result = readRegisters(ADDR, WHOAMI_REG, value);

  if (result != 0){
    Serial.print("I2C Bus Error: ");
    Serial.println(result);
  }

  if (value[0] == WHOAMI_VAL){
    Serial.println("Sensor returned correct WHOAMI, I2C bus is working as intended");
  } else {
    Serial.print("Sensor returned INCORRECT WHOAMI val, I2C bus not working. Returned: ");
    Serial.println(value[0]);
  }

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