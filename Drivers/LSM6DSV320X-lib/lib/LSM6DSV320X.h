//
// Created by Samuel Howes Manley on 11/7/25.
//

#ifndef LSM6DSV320X_H
#define LSM6DSV320X_H

#include <Arduino.h>
#include <Wire.h>

class LSM6DSV320X {
    public:

        LSM6DSV320X(int ADDR = 0x6A);

        void config();
        void read(int16_t* _raw_data, double* _data); // Passing in raw and processed (can log raw data in hex to compact the data)

    private:


        // Internal Functions
        byte readRegisters(byte _addr, byte _reg, byte* _value,byte _size = 1);
        byte writeRegister(byte _addr, byte _reg, byte _value);

        // Device Address
        const int IMU_ADDR  = 0x6A;

        // Registers
        const int whoami_reg = 0x0F;

        const int IMU_CTRL1               = 0x10;     // ACC:  Op mode, ODR
        const int IMU_CTRL2               = 0x11;     // GYRO: Op mode, ODR
        const int IMU_CTRL3               = 0x12;     //  N/A  for now
        const int IMU_CTRL4               = 0x13;     //  N/A  for now
        const int IMU_CTRL5               = 0x14;     //  N/A  for now
        const int IMU_CTRL6               = 0x15;     // GYRO: LPF1, FS Sel
        const int IMU_CTRL7               = 0x16;     // GYRO: Enbl LPF`
        const int IMU_CTRL8               = 0x17;     // ACC:  LPF2, FS Sel
        const int IMU_CTRL9               = 0x18;     // ACC:  Configuring Filters (Leave for Now)
        const int IMU_CTRL10              = 0x19;     //  IMU: Enbl dbg for Embedded functions and Self Testing (Leaving for now)

        const int IMU_OUTX_L_G            = 0x22;
        const int IMU_OUTX_H_G            = 0x23;
        const int IMU_OUTY_L_G            = 0x24;
        const int IMU_OUTY_H_G            = 0x25;
        const int IMU_OUTZ_L_G            = 0x26;
        const int IMU_OUTZ_H_G            = 0x27;
        const int IMU_OUTX_L_A            = 0x28;
        const int IMU_OUTX_H_A            = 0x29;
        const int IMU_OUTY_L_A            = 0x2A;
        const int IMU_OUTY_H_A            = 0x2B;
        const int IMU_OUTZ_L_A            = 0x2C;
        const int IMU_OUTZ_H_A            = 0x2D;

        const int IMU_UI_OUTX_L_A_OIS_HG  = 0x34;
        const int IMU_UI_OUTX_H_A_OIS_HG  = 0x35;
        const int IMU_UI_OUTY_L_A_OIS_HG  = 0x36;
        const int IMU_UI_OUTY_H_A_OIS_HG  = 0x37;
        const int IMU_UI_OUTZ_L_A_OIS_HG  = 0x38;
        const int IMU_UI_OUTZ_H_A_OIS_HG  = 0x39;

        const int IMU_CTRL2_XL_HG         = 0x4D;     // HG ACC: HG self-test sel (leave default for now)
        const int IMU_CTRL1_XL_HG         = 0x4E;     // HG ACC: Enbl output (reg: 34-39), hg user offset (out reg), ODR, FS Sel


        // Defining Values
        const int IMU_WHOAMI_VAL          = 0x72;

        // Data
        static constexpr byte imu_size = 18;    // 2 * 9 Axis
        byte imu_data[imu_size] = {};           // array for raw registers to be used as _value
};



#endif //LSM6DSV32X_H
