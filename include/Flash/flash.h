#pragma once
#include <cstddef>
#include <stdint.h>

class Flash {
    public:
        virtual ~Flash() = default;

        virtual bool init() = 0;
        virtual bool write(uint32_t address, const uint8_t* data, size_t length) = 0;
        virtual bool read(uint32_t address, uint8_t* buffer, size_t length) = 0;
        virtual bool erase(uint32_t address, size_t length) = 0;
};