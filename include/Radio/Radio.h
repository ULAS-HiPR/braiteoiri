#pragma once

#include <cstddef>
#include <cstdint>

class Radio {
public:
    virtual bool init() = 0;
    virtual bool send(const std::uint8_t* data, std::size_t len) = 0;
    virtual bool receive(std::uint8_t* data, std::size_t max_len, std::size_t* len) = 0;
    virtual ~Radio() = default;
};
