#ifndef LSM6DO_H
#define LSM6DO_H
#include <stdint.h>
#include "IMU.h"
#include <I2C/I2C_Handler.h>
#include <cstdio>


class LSM6DO : public IMU {
    public:
        explicit LSM6DO(SPI_Handler& spi_handler)
        : spi_handler(spi_handler) {}
        bool init() override;
        bool update(imu_data* out) override;
    
    private:
        void reset();
        void read_raw_accel(int16_t accel[3]);
        bool get_id();
        void update_range();
        SPI_Handler& spi_handler;
    
};

#endif // LSM6DO_H