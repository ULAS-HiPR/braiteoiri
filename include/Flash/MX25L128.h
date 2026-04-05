#pragma once
#include <cstdint>
#include <cstdio>
#include <Flash/flash.h>

class MX25L128 : public Flash {
    public:
        MX25L128();
        ~MX25L128() = default;
        bool init() override;
        bool write(uint32_t address, const uint8_t* data, size_t length) override;
        bool read(uint32_t address, uint8_t* buffer, size_t length) override;
        bool erase(uint32_t address, size_t length) override;

    private:
        uint8_t QPI_Enable = true;
};