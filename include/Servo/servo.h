#pragma once
#include <cstddef>
#include <stdint.h>

class Servo {
    public:
        virtual bool init();
        virtual bool set_poisiton(int8_t position);

    private:
        int8_t current_position;
};