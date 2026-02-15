#pragma once
#include <cstdint>
#include <data.h>
#include "../sensor.h"

class IMU : public Sensor<imu_data> {
public:
    virtual bool init() = 0;
    virtual bool update(imu_data* data) = 0; ;
};
