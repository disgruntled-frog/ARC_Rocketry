#ifndef LPS22HH_LIB_LIBRARY_H
#define LPS22HH_LIB_LIBRARY_H

#include "Arduino.h"

class LPS22HH
{
    public:
        LPS22HH();
        byte readRegisters(byte _addr, byte _reg, byte* _value,byte _size = 1);
        byte writeRegister(byte _addr, byte _reg, byte _value);
        void config_baro();
    void read_baro(byte* _value, int32_t& raw_baro, double& baro);


};

#endif //LPS22HH_LIB_LIBRARY_H
