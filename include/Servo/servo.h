#pragma once
#include <cstddef>
#include <stdint.h>

class Servo {
    public:
        virtual bool init() = 0;
        virtual bool set_positon(int8_t position) = 0;
        virtual ~Servo() = default;

    private:
        int8_t current_position;
};