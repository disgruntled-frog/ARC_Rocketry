//
// Created by Samuel Howes Manley on 11/4/25.
//

#ifndef LSM6DSV32X_H
#define LSM6DSV32X_H

#include <Arduino.h>

class LSM6DSV32X {
    public:
        LSM6DSV32X();

        void config();
        void read();
    private:


        // Internal Functions
        byte readRegisters(byte _addr, byte _reg, byte* _value,byte _size = 1);
        byte writeRegister(byte _addr, byte _reg, byte _value);

        // Device Address
        const int imu_addr  = 0x6A;

        // Registers
        const int whoami_reg = 0x0F;

        const int IMU_CTRL1       = 0x10;     // ACC:  Op mode, ODR
        const int IMU_CTRL2       = 0x11;     // GYRO: Op mode, ODR
        const int IMU_CTRL3       = 0x12;     //  N/A   for now
        const int IMU_CTRL4       = 0x13;     //  N/A   for now
        const int IMU_CTRL5       = 0x14;     //  N/A   for now
        const int IMU_CTRL6       = 0x15;     // GYRO: LPF1, FS Sel
        const int IMU_CTRL7       = 0x16;     // GYRO: Enbl LPF`
        const int IMU_CTRL8       = 0x17;     // ACC:  LPF2, FS Sel
        const int IMU_CTRL9       = 0x18;     // ACC:  Configuring Filters (Leave for Now)
        const int IMU_CTRL10      = 0x19;     //  IMU:  Enbl dbg for Embedded functions and Self Testing (Leaving for now)

        const int IMU_OUTX_L_G    = 0x22;
        const int IMU_OUTX_H_G    = 0x23;
        const int IMU_OUTY_L_G    = 0x24;
        const int IMU_OUTY_H_G    = 0x25;
        const int IMU_OUTZ_L_G    = 0x26;
        const int IMU_OUTZ_H_G    = 0x27;
        const int IMU_OUTX_L_A    = 0x28;
        const int IMU_OUTX_H_A    = 0x29;
        const int IMU_OUTY_L_A    = 0x2A;
        const int IMU_OUTY_H_A    = 0x2B;
        const int IMU_OUTZ_L_A    = 0x2C;
        const int IMU_OUTZ_H_A    = 0x2D;

        // Values
        const int whoami_val      = 0x70;

        // Data
        static constexpr byte imu_size = 12;    // 2 * 6 Axis
        byte imu_data[imu_size] = {};           // array for raw registers to be used as _value
};



#endif //LSM6DSV32X_H
