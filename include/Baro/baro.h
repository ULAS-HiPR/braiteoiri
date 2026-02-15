#pragma once
#include "../sensor.h"
#include <stdint.h>
#include <data.h>

class Baro : public Sensor<baro_data> {
public:
    virtual bool init() = 0;
    bool update(baro_data* data) override;
};
