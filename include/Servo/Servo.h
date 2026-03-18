#pragma once
#include <cstdint>
#include <data.h>

class Servo {
public:
    virtual bool init() = 0;
    virtual bool turn() = 0;
    virtual bool set_position(uint16_t position) = 0;
};
