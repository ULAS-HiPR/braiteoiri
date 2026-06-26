#pragma once

#include <data.h>
#include "../sensor.h"

class GNSS : public Sensor<gps_data> {
public:
    virtual bool init() = 0;
    virtual bool update(gps_data* data) = 0;
    virtual bool poll_navigation_satellites() = 0;
};
