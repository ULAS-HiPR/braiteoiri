#pragma once
#include <cstdint>
#include "../sensor.h"

//12 bytes
struct accle_data
{
    float x{0};
    float y{0};
    float z{0};
};

//6 bytes
struct gyro_data
{
    int16_t x{0};
    int16_t y{0};
    int16_t z{0};
};

// 18 bytes
struct imu_data
{
    accle_data accel;
    gyro_data gyro;
    uint64_t t_us;
};

class IMU : Sensor<imu_data> {
public:
    virtual bool init() = 0;
    bool update(imu_data* data) override;
};
