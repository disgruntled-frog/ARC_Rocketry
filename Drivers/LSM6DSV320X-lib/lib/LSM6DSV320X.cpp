//
// Created by Samuel Howes Manley on 11/7/25.
//

#include "LSM6DSV320X.h"

LSM6DSV320X::LSM6DSV320X(const int ADDR) : IMU_ADDR(ADDR){}

void LSM6DSV320X::config(){
    // This will be default configuration for everyone, future plans to add customization
      byte results = writeRegister(IMU_ADDR, IMU_CTRL1, B00001000);   // High-performance, ODR 480 Hz
      results = writeRegister(IMU_ADDR, IMU_CTRL2, B00001000);        // High-performance, ODR 480 Hz
      results = writeRegister(IMU_ADDR, IMU_CTRL6, B00001101);        // LPF1: 175, FS +/- 4000 dps
      results = writeRegister(IMU_ADDR, IMU_CTRL8, B00000011);        // BW: ODR/4, FS +/- 16 g
      results = writeRegister(IMU_ADDR, IMU_CTRL9, B00001000);        // Selecting output from ACC LPF2. Come back to this one to tune fitlering
      results = writeRegister(IMU_ADDR, IMU_CTRL1_XL_HG, B11011000);  // HG ODR 480 Hz, +/- 32 g, Enble Output, Enble user offset on out reg
    }

void LSM6DSV320X::read(int16_t* _raw_data, double* _data){

    readRegisters(IMU_ADDR, IMU_OUTX_L_G, imu_data, 12);              // _value == imu_reg_data
    readRegisters(IMU_ADDR, IMU_UI_OUTX_L_A_OIS_HG, imu_data+12, 6);  // reading in the last 6 bytes into _value

    // Byte Concatenation: _raw_data -> [gyroX,gyroY,gyroZ,accX,accY,accZ,hg_accx,hg_accy,hg_accz]
    _raw_data[0] = (imu_data[1]<<8)|(imu_data[0]);
    _raw_data[1] = (imu_data[3]<<8)|(imu_data[2]);
    _raw_data[2] = (imu_data[5]<<8)|(imu_data[4]);

    _raw_data[3] = (imu_data[7]<<8)|(imu_data[6]);
    _raw_data[4] = (imu_data[9]<<8)|(imu_data[8]);
    _raw_data[5] = (imu_data[11]<<8)|(imu_data[10]);

    _raw_data[6] = (imu_data[13]<<8)|(imu_data[12]);
    _raw_data[7] = (imu_data[15]<<8)|(imu_data[14]);
    _raw_data[8] = (imu_data[17]<<8)|(imu_data[16]);

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

byte LSM6DSV320X::readRegisters(byte _addr, byte _reg, byte* _value, byte _size){
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

byte LSM6DSV320X::writeRegister(byte _addr, byte _reg, byte _value){
    // byte _addr: address of the I2C sensor you wish to communicate with
    // byte _reg: address of the register within said I2C sensor you wish to write to
    // byte _value: byte you wish to write to said register

    Wire.beginTransmission(_addr);
    Wire.write(_reg);
    Wire.write(_value);
    return Wire.endTransmission();
}