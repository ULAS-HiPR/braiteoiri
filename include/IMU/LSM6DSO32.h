#ifndef LSM6DSO32_H
#define LSM6DSO32_H

#include "IMU.h"
#include <SPI/SPI_Handler.h>
#include <cstdint>

enum lsm6dso32_accel_range_t : std::uint8_t {
    LSM6DSO32_ACCEL_RANGE_4_G = 0x00,
    LSM6DSO32_ACCEL_RANGE_32_G = 0x04,
    LSM6DSO32_ACCEL_RANGE_8_G = 0x08,
    LSM6DSO32_ACCEL_RANGE_16_G = 0x0C,
};

enum lsm6dso32_gyro_range_t : std::uint8_t {
    LSM6DSO32_GYRO_RANGE_250_DPS = 0x00,
    LSM6DSO32_GYRO_RANGE_125_DPS = 0x02,
    LSM6DSO32_GYRO_RANGE_500_DPS = 0x04,
    LSM6DSO32_GYRO_RANGE_1000_DPS = 0x08,
    LSM6DSO32_GYRO_RANGE_2000_DPS = 0x0C,
};

class LSM6DSO32 : public IMU {
public:
    explicit LSM6DSO32(SPI_Handler& spi_handler)
        : spi_handler(spi_handler) {}

    bool init() override;
    bool update(imu_data* data) override;

private:
    bool reset();
    bool configure();
    bool device_present();
    std::uint8_t read_register(std::uint8_t reg);
    void read_registers(std::uint8_t reg, std::uint8_t* data, std::uint16_t len);
    void write_register(std::uint8_t reg, std::uint8_t value);
    float accel_scale_g_per_lsb() const;
    float gyro_scale_dps_per_lsb() const;

    SPI_Handler& spi_handler;
    lsm6dso32_accel_range_t accel_range{LSM6DSO32_ACCEL_RANGE_32_G};
    lsm6dso32_gyro_range_t gyro_range{LSM6DSO32_GYRO_RANGE_2000_DPS};
    bool initialized{false};
};

#endif // LSM6DSO32_H
