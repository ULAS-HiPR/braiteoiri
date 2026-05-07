#pragma once
#include <cstddef>
#include <stdint.h>

class Servo {
    public:
        virtual bool init() = 0;
        virtual bool set_position(int16_t position) = 0;
        virtual ~Servo() = default;
        int16_t current_position;
};
