//
// Created by Samuel Howes Manley on 11/4/25.
//

#include "LSM6DSV32X.h"

LSM6DSV32X::LSM6DSV32X(const int ADDR) : IMU_ADDR(ADDR){}

void LSM6DSV32X::config(){
    // This will be default configuration for everyone, future plans to add customization
    byte results = writeRegister(IMU_ADDR, IMU_CTRL1, B00001000); // High-performance, ODR 480 Hz
    results = writeRegister(IMU_ADDR, IMU_CTRL2, B00001000); // High-performance, ODR 480 Hz
    results = writeRegister(IMU_ADDR, IMU_CTRL6, B00001100); // LPF1: 175, FS +/- 4000 dps
    results = writeRegister(IMU_ADDR, IMU_CTRL8, B00000111); // BW: ODR/4, FS +/- 32 g
    results = writeRegister(IMU_ADDR, IMU_CTRL9, B00001000); // Selecting output from ACC LPF2. Come back to this one to tune fitlering
}

void LSM6DSV32X::read(int16_t* _raw_data, double* _data){

    readRegisters(IMU_ADDR, IMU_OUTX_L_G, imu_data, imu_size);   //_value == imu_data

    // Byte Concatenation: _raw_data -> [gyroX,gyroY,gyroZ,accX,accY,accZ]
    _raw_data[0] = (imu_data[1]<<8)|(imu_data[0]);
    _raw_data[1] = (imu_data[3]<<8)|(imu_data[2]);
    _raw_data[2] = (imu_data[5]<<8)|(imu_data[4]);

    _raw_data[3] = (imu_data[7]<<8)|(imu_data[6]);
    _raw_data[4] = (imu_data[9]<<8)|(imu_data[8]);
    _raw_data[5] = (imu_data[11]<<8)|(imu_data[10]);

    // Convert Gyro into dps
    double fs_dps = 4000;
    double max_16_signed = 32767;
    double gyro_scale_factor = fs_dps / max_16_signed;

    for (int i=0; i<3; i++){
        _data[i] = _raw_data[i] * gyro_scale_factor;
    }

    // Convert Acc into g
    double fs_g = 32;
    double acc_scale_factor = fs_g / max_16_signed;

    for (int i=3; i<6; i++){
        _data[i] = _raw_data[i] * acc_scale_factor;
    }

}

byte LSM6DSV32X::readRegisters(byte _addr, byte _reg, byte* _value, byte _size){
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

byte LSM6DSV32X::writeRegister(byte _addr, byte _reg, byte _value){
    // byte _addr: address of the I2C sensor you wish to communicate with
    // byte _reg: address of the register within said I2C sensor you wish to write to
    // byte _value: byte you wish to write to said register

    Wire.beginTransmission(_addr);
    Wire.write(_reg);
    Wire.write(_value);
    return Wire.endTransmission();
}