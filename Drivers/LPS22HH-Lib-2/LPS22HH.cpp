//
// Created by Samuel Howes Manley on 10/26/25.
//

#include "LPS22HH.h"

LPS22HH::LPS22HH(const int ADDR) : BARO_ADDR(ADDR) {}

void LPS22HH::config(){
    // This will be default configuration for everyone, future plans to add customization
    byte results = writeRegister(BARO_ADDR, ctrl1_reg, 0b01110000); // ODR 200 Hz, No Low pass, Coninuous Mode, 4-wire SPI
}

void LPS22HH::read(int32_t& raw_baro, double& baro){
    // Passing in raw and baro so we can log easier

    readRegisters(BARO_ADDR,out_xl_reg,baro_data,baro_size);

    // Byte Concatenation
    raw_baro = (baro_data[2]<<16)|(baro_data[1]<<8)|(baro_data[0]);  // baro_data == [LSB,midSB,MSB]
    // Convert to hPa
    baro = raw_baro / 4096.0;

}

byte LPS22HH::readRegisters(byte _addr, byte _reg, byte* _value, byte _size){
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

byte LPS22HH::writeRegister(byte _addr, byte _reg, byte _value){
    // byte _addr: address of the I2C sensor you wish to communicate with
    // byte _reg: address of the register within said I2C sensor you wish to write to
    // byte _value: byte you wish to write to said register

    Wire.beginTransmission(_addr);
    Wire.write(_reg);
    Wire.write(_value);
    return Wire.endTransmission();
}