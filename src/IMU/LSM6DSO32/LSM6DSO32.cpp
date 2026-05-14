#include <IMU/LSM6DSO32.h>

#include <cmath>
#include <cstdint>
#include <limits>

namespace {

constexpr std::uint8_t kWhoAmIReg = 0x0F;
constexpr std::uint8_t kCtrl1XlReg = 0x10;
constexpr std::uint8_t kCtrl2GReg = 0x11;
constexpr std::uint8_t kCtrl3CReg = 0x12;
constexpr std::uint8_t kOutTempLReg = 0x20;

constexpr std::uint8_t kWhoAmIValue = 0x6C;
constexpr std::uint8_t kCtrl3CReset = 0x01;
constexpr std::uint8_t kCtrl3CBlockDataUpdate = 0x40;
constexpr std::uint8_t kCtrl3CIncrementAddress = 0x04;
constexpr std::uint8_t kOutputDataRate416Hz = 0x60;

constexpr std::uint16_t kBurstReadLength = 14;

struct RawSample {
    std::int16_t temperature;
    std::int16_t gyro_x;
    std::int16_t gyro_y;
    std::int16_t gyro_z;
    std::int16_t accel_x;
    std::int16_t accel_y;
    std::int16_t accel_z;
};

std::int16_t combine_bytes(std::uint8_t low, std::uint8_t high)
{
    return static_cast<std::int16_t>(
        (static_cast<std::uint16_t>(high) << 8U) |
        static_cast<std::uint16_t>(low));
}

std::int16_t clamp_to_int16(float value)
{
    if (value > static_cast<float>(std::numeric_limits<std::int16_t>::max())) {
        return std::numeric_limits<std::int16_t>::max();
    }

    if (value < static_cast<float>(std::numeric_limits<std::int16_t>::min())) {
        return std::numeric_limits<std::int16_t>::min();
    }

    return static_cast<std::int16_t>(std::lround(value));
}

} // namespace

bool LSM6DSO32::init()
{
    if (!reset()) {
        return false;
    }

    if (!device_present()) {
        return false;
    }

    if (!configure()) {
        return false;
    }

    initialized = true;
    return true;
}

bool LSM6DSO32::update(imu_data* data)
{
    if ((!initialized) || (data == nullptr)) {
        return false;
    }

    std::uint8_t buffer[kBurstReadLength] = {};
    read_registers(kOutTempLReg, buffer, kBurstReadLength);

    const RawSample sample{
        combine_bytes(buffer[0], buffer[1]),
        combine_bytes(buffer[2], buffer[3]),
        combine_bytes(buffer[4], buffer[5]),
        combine_bytes(buffer[6], buffer[7]),
        combine_bytes(buffer[8], buffer[9]),
        combine_bytes(buffer[10], buffer[11]),
        combine_bytes(buffer[12], buffer[13]),
    };

    const float accel_scale = accel_scale_g_per_lsb();
    const float gyro_scale = gyro_scale_dps_per_lsb();

    data->acceleration.x = static_cast<float>(sample.accel_x) * accel_scale;
    data->acceleration.y = static_cast<float>(sample.accel_y) * accel_scale;
    data->acceleration.z = static_cast<float>(sample.accel_z) * accel_scale;

    data->gyro.x = clamp_to_int16(static_cast<float>(sample.gyro_x) * gyro_scale);
    data->gyro.y = clamp_to_int16(static_cast<float>(sample.gyro_y) * gyro_scale);
    data->gyro.z = clamp_to_int16(static_cast<float>(sample.gyro_z) * gyro_scale);

    data->temperature = static_cast<int>(std::lround(
        25.0f + (static_cast<float>(sample.temperature) / 256.0f)));

    return true;
}

bool LSM6DSO32::reset()
{
    write_register(kCtrl3CReg, kCtrl3CReset);

    for (int attempt = 0; attempt < 10; ++attempt) {
        spi_handler.delay_ms(1);
        if ((read_register(kCtrl3CReg) & kCtrl3CReset) == 0U) {
            return true;
        }
    }

    return false;
}

bool LSM6DSO32::configure()
{
    write_register(
        kCtrl3CReg,
        static_cast<std::uint8_t>(kCtrl3CBlockDataUpdate | kCtrl3CIncrementAddress));
    write_register(
        kCtrl1XlReg,
        static_cast<std::uint8_t>(kOutputDataRate416Hz | static_cast<std::uint8_t>(accel_range)));
    write_register(
        kCtrl2GReg,
        static_cast<std::uint8_t>(kOutputDataRate416Hz | static_cast<std::uint8_t>(gyro_range)));

    return true;
}

bool LSM6DSO32::device_present()
{
    return read_register(kWhoAmIReg) == kWhoAmIValue;
}

std::uint8_t LSM6DSO32::read_register(std::uint8_t reg)
{
    std::uint8_t value = 0U;
    read_registers(reg, &value, 1U);
    return value;
}

void LSM6DSO32::read_registers(std::uint8_t reg, std::uint8_t* data, std::uint16_t len)
{
    spi_handler.read(0, reg, data, len);
}

void LSM6DSO32::write_register(std::uint8_t reg, std::uint8_t value)
{
    spi_handler.write(0, reg, &value, 1U);
}

float LSM6DSO32::accel_scale_g_per_lsb() const
{
    switch (accel_range) {
        case LSM6DSO32_ACCEL_RANGE_4_G:
            return 0.000122f;
        case LSM6DSO32_ACCEL_RANGE_8_G:
            return 0.000244f;
        case LSM6DSO32_ACCEL_RANGE_16_G:
            return 0.000488f;
        case LSM6DSO32_ACCEL_RANGE_32_G:
        default:
            return 0.000976f;
    }
}

float LSM6DSO32::gyro_scale_dps_per_lsb() const
{
    switch (gyro_range) {
        case LSM6DSO32_GYRO_RANGE_125_DPS:
            return 0.004375f;
        case LSM6DSO32_GYRO_RANGE_250_DPS:
            return 0.008750f;
        case LSM6DSO32_GYRO_RANGE_500_DPS:
            return 0.017500f;
        case LSM6DSO32_GYRO_RANGE_1000_DPS:
            return 0.035000f;
        case LSM6DSO32_GYRO_RANGE_2000_DPS:
        default:
            return 0.070000f;
    }
}
