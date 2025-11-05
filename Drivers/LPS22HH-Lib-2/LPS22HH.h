//
// Created by Samuel Howes Manley on 10/26/25.
//

#ifndef LPS22HH_H
#define LPS22HH_H

#include "Arduino.h"
#include <Wire.h>

class LPS22HH {
    public:
        LPS22HH();

        void config_baro();
        void read_baro(int32_t& raw_baro, double& baro);

    private:

        // Internal Functions
        byte readRegisters(byte _addr, byte _reg, byte* _value,byte _size = 1);
        byte writeRegister(byte _addr, byte _reg, byte _value);

        // Device Address
        const int baro_addr  = 0x5C;

        // Registers
        const int whoami_reg = 0x0F;
        const int ctrl1_reg  = 0x10;
        const int out_xl_reg = 0x28;
        const int out_l_reg  = 0x29;
        const int out_h_reg  = 0x2A;

        // Values
        const int whoami_val = 0xB3;

        // Data
        static constexpr byte baro_size = 3;
        byte baro_data[baro_size] = {};

};



#endif //LPS22HH_H
