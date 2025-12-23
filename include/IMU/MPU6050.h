#ifndef MPU6050_H
#define MPU6050_H
#include <stdint.h>
#include "IMU.h"
#include <I2C_Handler.h>

//most code from offical raspberry pi pico example
typedef enum {
    MPU6050_RANGE_2_G = 0,  ///< +/- 2g (default value)
    MPU6050_RANGE_4_G = 1,  ///< +/- 4g
    MPU6050_RANGE_8_G = 2,  ///< +/- 8g
    MPU6050_RANGE_16_G = 3, ///< +/- 16g
  } mpu6050_accel_range_t;


class MPU6050 : public IMU {
    public:
        explicit MPU6050(I2C_Handler& i2c_handler)
        : i2c_handler(i2c_handler) {}

        bool init() override;
        bool read(imu_data& out) override;
    
    private:
        void reset();
        void read_raw_accel(int16_t accel[3]);
        bool get_id();
        void update_range();
        void set_range(mpu6050_accel_range_t accel_range);
        mpu6050_accel_range_t accel_range{MPU6050_RANGE_2_G};
        uint8_t const MPU6050_ADDRESS = 0x68; // I2C address of the MPU6050
        uint8_t const MPU6050_ACCEL_OUT = 0x3B; // Register address for accelerometer X-axis high byte
        uint8_t const MPU6050_ACCEL_CONFIG = 0x1C; // Register address for accelerometer configuration
        uint8_t const MPU6050_ID_LOC = 0x75; // Register address for device ID
        uint8_t const MPU6050_ID = 0x68; // Expected device ID
        I2C_Handler& i2c_handler;
    
};

#endif // MPU6050_H