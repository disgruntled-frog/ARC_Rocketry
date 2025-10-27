//
// Created by Samuel Howes Manley on 10/26/25.
//

#ifndef LPS22HH_H
#define LPS22HH_H

#include "Arduino.h"

class LPS22HH {
    public:
        LPS22HH();

        void config_baro();
        void read_baro(double& baro);
        double get_alt();
    private:
        double alt;

        // Internal Functions
        byte readRegisters(byte _addr, byte _reg, byte* _value,byte _size = 1);
        byte writeRegister(byte _addr, byte _reg, byte _value);

        // Registers
        int whoami_reg = 0x0F;
        int ctrl1_reg  = 0x10;
        int out_xl_reg = 0x28;
        int out_l_reg  = 0x29;
        int out_h_reg  = 0x2A;

        // Values
        int whoami_val = 0xB3;


};



#endif //LPS22HH_H
