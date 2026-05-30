#include <Baro/MS5607.h>

#include <cmath>
#include <cstdint>
#include <limits>

namespace {

constexpr std::uint8_t kCommandReset = 0x1E;
constexpr std::uint8_t kCommandConvertD1Base = 0x40;
constexpr std::uint8_t kCommandConvertD2Base = 0x50;
constexpr std::uint8_t kCommandAdcRead = 0x00;
constexpr std::uint8_t kCommandPromReadBase = 0xA0;

constexpr float kFallbackSeaLevelPressurePa = 101325.0f;
constexpr float kAltitudeExponent = 0.19029495f;

int conversion_delay_for_osr(ms5607_osr_t osr)
{
    switch (osr) {
        case MS5607_OSR_256:
            return 1;
        case MS5607_OSR_512:
            return 2;
        case MS5607_OSR_1024:
            return 3;
        case MS5607_OSR_2048:
            return 5;
        case MS5607_OSR_4096:
        default:
            return 10;
    }
}

} // namespace

bool MS5607::init()
{
    send_command(kCommandReset);
    spi_handler.delay_ms(3);

    if (!read_prom()) {
        return false;
    }

    const std::uint8_t stored_crc = static_cast<std::uint8_t>(prom[7] & 0x0F);
    if (calculate_crc4() != stored_crc) {
        return false;
    }

    std::int32_t pressure_pa = 0;
    float temperature_c = 0.0f;
    if (!read_compensated_sample(&pressure_pa, &temperature_c)) {
        return false;
    }

    reference_pressure_pa = static_cast<float>(pressure_pa);
    reference_pressure_valid = pressure_pa > 0;
    initialized = true;
    (void)temperature_c;
    return true;
}

bool MS5607::update(baro_data* data)
{
    if ((!initialized) || (data == nullptr)) {
        return false;
    }

    std::int32_t pressure_pa = 0;
    float temperature_c = 0.0f;
    if (!read_compensated_sample(&pressure_pa, &temperature_c)) {
        return false;
    }

    data->pressure = pressure_pa;
    data->temperature = temperature_c;
    data->altitude = pressure_to_altitude(static_cast<float>(pressure_pa));
    return true;
}

void MS5607::send_command(std::uint8_t command)
{
    std::uint8_t unused = 0U;

    spi_handler.cs_select(0);
    spi_handler.write_no_cs(command, &unused, 0U);
    spi_handler.cs_deselect(0);
}

void MS5607::read_command(std::uint8_t command, std::uint8_t* data, std::uint16_t len)
{
    // The MS5607 SPI protocol is command-based, so the transport layer needs to
    // clock out the command byte exactly as provided here. Do not use the
    // register helper: it ORs the command with the MSB read bit.
    spi_handler.cs_select(0);
    (void)(spi_handler.transmit(&command, 1U) && spi_handler.receive(data, len));
    spi_handler.cs_deselect(0);
}

bool MS5607::read_prom()
{
    for (std::uint8_t index = 0; index < prom.size(); ++index) {
        prom[index] = read_prom_word(index);
    }

    return true;
}

std::uint16_t MS5607::read_prom_word(std::uint8_t index)
{
    std::uint8_t buffer[2] = {};
    read_command(
        static_cast<std::uint8_t>(kCommandPromReadBase + (index * 2U)),
        buffer,
        2U);

    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(buffer[0]) << 8U) |
        static_cast<std::uint16_t>(buffer[1]));
}

std::uint8_t MS5607::calculate_crc4() const
{
    std::array<std::uint16_t, 8> local_prom = prom;
    local_prom[7] = static_cast<std::uint16_t>(local_prom[7] & 0xFF00U);

    std::uint16_t remainder = 0U;
    constexpr std::uint16_t polynomial = 0x3000U;

    for (std::size_t index = 0; index < 16U; ++index) {
        const std::size_t prom_index = index / 2U;

        if ((index % 2U) == 0U) {
            remainder = static_cast<std::uint16_t>(
                remainder ^ static_cast<std::uint16_t>(local_prom[prom_index] >> 8U));
        } else {
            remainder = static_cast<std::uint16_t>(
                remainder ^ static_cast<std::uint16_t>(local_prom[prom_index] & 0x00FFU));
        }

        for (std::size_t bit = 0; bit < 8U; ++bit) {
            if ((remainder & 0x8000U) != 0U) {
                remainder = static_cast<std::uint16_t>(
                    static_cast<std::uint16_t>(remainder << 1U) ^ polynomial);
            } else {
                remainder = static_cast<std::uint16_t>(remainder << 1U);
            }
        }
    }

    return static_cast<std::uint8_t>((remainder >> 12U) & 0x0FU);
}

std::uint32_t MS5607::convert_and_read(std::uint8_t command, int delay_ms)
{
    std::uint8_t buffer[3] = {};

    send_command(command);
    spi_handler.delay_ms(delay_ms);
    read_command(kCommandAdcRead, buffer, 3U);

    return
        (static_cast<std::uint32_t>(buffer[0]) << 16U) |
        (static_cast<std::uint32_t>(buffer[1]) << 8U) |
        static_cast<std::uint32_t>(buffer[2]);
}

bool MS5607::read_compensated_sample(std::int32_t* pressure_pa, float* temperature_c)
{
    if ((pressure_pa == nullptr) || (temperature_c == nullptr)) {
        return false;
    }

    const int delay_ms = conversion_delay_for_osr(osr);
    const std::uint32_t d1 = convert_and_read(
        static_cast<std::uint8_t>(kCommandConvertD1Base | static_cast<std::uint8_t>(osr)),
        delay_ms);
    const std::uint32_t d2 = convert_and_read(
        static_cast<std::uint8_t>(kCommandConvertD2Base | static_cast<std::uint8_t>(osr)),
        delay_ms);

    const std::int64_t d_t = static_cast<std::int64_t>(d2) -
        (static_cast<std::int64_t>(prom[5]) << 8U);
    std::int64_t temp = 2000 +
        ((d_t * static_cast<std::int64_t>(prom[6])) >> 23U);
    std::int64_t off = (static_cast<std::int64_t>(prom[2]) << 17U) +
        ((static_cast<std::int64_t>(prom[4]) * d_t) >> 6U);
    std::int64_t sens = (static_cast<std::int64_t>(prom[1]) << 16U) +
        ((static_cast<std::int64_t>(prom[3]) * d_t) >> 7U);

    std::int64_t t2 = 0;
    std::int64_t off2 = 0;
    std::int64_t sens2 = 0;

    if (temp < 2000) {
        const std::int64_t temp_delta = temp - 2000;
        t2 = (d_t * d_t) >> 31U;
        off2 = (61 * temp_delta * temp_delta) >> 4U;
        sens2 = 2 * temp_delta * temp_delta;

        if (temp < -1500) {
            const std::int64_t low_temp_delta = temp + 1500;
            off2 += 15 * low_temp_delta * low_temp_delta;
            sens2 += 8 * low_temp_delta * low_temp_delta;
        }
    }

    temp -= t2;
    off -= off2;
    sens -= sens2;

    const std::int64_t pressure =
        ((((static_cast<std::int64_t>(d1) * sens) >> 21U) - off) >> 15U);

    if ((pressure < static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::min())) ||
        (pressure > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()))) {
        return false;
    }

    *pressure_pa = static_cast<std::int32_t>(pressure);
    *temperature_c = static_cast<float>(temp) / 100.0f;
    return true;
}

float MS5607::pressure_to_altitude(float pressure_pa) const
{
    const float base_pressure = reference_pressure_valid
        ? reference_pressure_pa
        : kFallbackSeaLevelPressurePa;

    if ((pressure_pa <= 0.0f) || (base_pressure <= 0.0f)) {
        return 0.0f;
    }

    return 44330.0f * (1.0f - std::pow(pressure_pa / base_pressure, kAltitudeExponent));
}
