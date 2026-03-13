#pragma once
#include <cstdint>
#include <data.h>

class Radio {
public:
    virtual bool init() = 0;
    virtual bool send() = 0;
    virtual bool receive() = 0;
};
