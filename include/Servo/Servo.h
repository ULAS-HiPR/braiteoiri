#pragma once
#include <cstdint>
#include <data.h>

class Servo {
public:
    virtual bool init() = 0;
    virtual bool turn() = 0;
};
