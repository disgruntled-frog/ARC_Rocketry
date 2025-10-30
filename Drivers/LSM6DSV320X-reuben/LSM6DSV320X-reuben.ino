#include <Wire.h>


#define IMU_TARGET_ADDR 0x6A 

#define IMU_WHOAMI_REG_ADDR 0x0F
#define IMU_WHOAMI_EXPECTED_VALUE 0x73 // According to datasheet, 73 is the expected value


#define I2C_SCL_GPIO_PIN 21
#define I2C_SDA_GPIO_PIN 22

#define LOOP_DELAY_TIME_MS 5000
#define DEFAULT_SERIAL_BAUD_RATE 9600


#define IMU_XL_OP_REG_ADDR  0x10
// Control register 1 input values
typedef enum {
    IMU_XL_OP_MODE_HIGH_PERFORMANCE = 0b000,  
    IMU_XL_OP_MODE_HIGH_ACCURACY    = 0b001,
    IMU_XL_OP_MODE_RESERVED         = 0b010,
    IMU_XL_OP_MODE_ODR_TRIGGERED    = 0b011,
    IMU_XL_OP_MODE_LOW_POWER_1      = 0b100,  
    IMU_XL_OP_MODE_LOW_POWER_2      = 0b101,  
    IMU_XL_OP_MODE_LOW_POWER_3      = 0b110,  
    IMU_XL_OP_MODE_NORMAL           = 0b111
} imu_xl_op_mode;


#define IMU_G_OP_REG_ADDR 0x11
typedef enum {
    IMU_G_OP_MODE_HIGH_PERFORMANCE = 0b000,  // default
    IMU_G_OP_MODE_HIGH_ACCURACY    = 0b001,
    IMU_G_OP_MODE_RESERVED_1       = 0b010,
    IMU_G_OP_MODE_ODR_TRIGGERED    = 0b011,
    IMU_G_OP_MODE_SLEEP            = 0b100,
    IMU_G_OP_MODE_LOW_POWER        = 0b101,
    IMU_G_OP_MODE_RESERVED_2       = 0b110,
    IMU_G_OP_MODE_RESERVED_3       = 0b111
} imu_g_op_mode;


void setup() {

  Wire.begin(); 
  Serial.begin(DEFAULT_SERIAL_BAUD_RATE);

}

void loop() {

  if (_verify_register_value(IMU_WHOAMI_REG_ADDR, IMU_WHOAMI_EXPECTED_VALUE, true)){
    Serial.println("I2C communication established!");
  } 
  else{
    Serial.print("WHOAMI does not match. Expected: 0x73, Got: ");
    return;
  }

  set_accelerometer_mode(IMU_XL_OP_MODE_LOW_POWER_1);
  set_gyroscope_mode(IMU_G_OP_MODE_LOW_POWER);
  delay(LOOP_DELAY_TIME_MS);
}


void set_accelerometer_mode(imu_xl_op_mode mode){
  _writeRegister(IMU_TARGET_ADDR, IMU_XL_OP_REG_ADDR, mode);
  _verify_register_value(IMU_XL_OP_REG_ADDR, mode, true);
}

void set_gyroscope_mode(imu_g_op_mode mode){
  _writeRegister(IMU_TARGET_ADDR, IMU_G_OP_REG_ADDR, mode);
  _verify_register_value(IMU_G_OP_REG_ADDR, mode, true);
}


bool _verify_register_value(byte register_address, byte expected_value, bool debug){
  byte register_value[1];
  _readRegisters(IMU_TARGET_ADDR, register_address, register_value, 1);

  if (debug){
    Serial.print("Expected register value: ");
    Serial.println(expected_value, HEX);
    Serial.print("Given register value ");
    Serial.println(*register_value, HEX);
  }

  return (*register_value == expected_value);
}


byte _readRegisters(byte _addr, byte _reg, byte* _value, byte _size){
  // byte _addr: address of the I2C sensor you wish to communicate with
  // byte _reg: address of the register within said I2C sensor you wish to read from
  // byte* _value: pointer to the array you will store the value(s) read
  // byte _size: size of the value array
  // byte returnval: error code, 0 is normal operation


  Wire.beginTransmission(_addr);
  Wire.write(_reg);
  Wire.endTransmission(false);
  Wire.requestFrom(_addr, _size);

  while(Wire.available()){
    *_value = Wire.read();
    _value++;
  }
 
  return 0;

}


byte _writeRegister(byte _addr, byte _reg, byte _value){
  // byte _addr: address of the I2C sensor you wish to communicate with
  // byte _reg: address of the register within said I2C sensor you wish to write to
  // byte _value: byte you wish to write to said register

  Wire.beginTransmission(_addr);
  Wire.write(_reg);
  Wire.write(_value);
  return Wire.endTransmission();
}




