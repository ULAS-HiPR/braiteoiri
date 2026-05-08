#include <Baro/MS5607.h>
#include <SPI/SPI_Handler.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

std::uint8_t compute_crc4(std::uint16_t* prom)
{
    std::uint16_t local_prom[8] = {};
    for (int index = 0; index < 8; ++index) {
        local_prom[index] = prom[index];
    }

    local_prom[7] = static_cast<std::uint16_t>(local_prom[7] & 0xFF00U);

    std::uint16_t remainder = 0U;
    for (int index = 0; index < 16; ++index) {
        if ((index % 2) == 0) {
            remainder = static_cast<std::uint16_t>(
                remainder ^ static_cast<std::uint16_t>(local_prom[index / 2] >> 8U));
        } else {
            remainder = static_cast<std::uint16_t>(
                remainder ^ static_cast<std::uint16_t>(local_prom[index / 2] & 0x00FFU));
        }

        for (int bit = 0; bit < 8; ++bit) {
            if ((remainder & 0x8000U) != 0U) {
                remainder = static_cast<std::uint16_t>((remainder << 1U) ^ 0x3000U);
            } else {
                remainder = static_cast<std::uint16_t>(remainder << 1U);
            }
        }
    }

    return static_cast<std::uint8_t>((remainder >> 12U) & 0x0FU);
}

class MockSPI final : public SPI_Handler {
public:
    MockSPI()
        : prom_words{0, 46372, 43981, 29059, 27842, 31553, 28165, 0}
    {
        prom_words[7] = compute_crc4(prom_words.data());
    }

    void read(int, std::uint8_t, std::uint8_t*, std::uint16_t) override {}

    void read_no_cs(std::uint8_t reg, std::uint8_t* buf, std::uint16_t len) override
    {
        if ((reg >= 0xA0U) && (reg <= 0xAEU) && (len == 2U)) {
            const std::uint8_t index = static_cast<std::uint8_t>((reg - 0xA0U) / 2U);
            buf[0] = static_cast<std::uint8_t>(prom_words[index] >> 8U);
            buf[1] = static_cast<std::uint8_t>(prom_words[index] & 0x00FFU);
            return;
        }

        if ((reg == 0x00U) && (len == 3U)) {
            const std::uint32_t sample = (last_conversion_command & 0x10U) != 0U
                ? 8077636U
                : 6465444U;
            buf[0] = static_cast<std::uint8_t>(sample >> 16U);
            buf[1] = static_cast<std::uint8_t>(sample >> 8U);
            buf[2] = static_cast<std::uint8_t>(sample & 0x0000FFU);
            return;
        }

        std::fill(buf, buf + len, 0U);
    }

    void write(int, std::uint8_t, std::uint8_t*, std::uint16_t) override {}

    void write_no_cs(std::uint8_t reg, const std::uint8_t*, std::uint16_t) override
    {
        last_conversion_command = reg;
        commands.push_back(reg);
    }

    void cs_select(int) override {}
    void cs_deselect(int) override {}
    void delay_ms(int ms) override { delays.push_back(ms); }

    std::vector<std::uint16_t> prom_words;
    std::vector<std::uint8_t> commands;
    std::vector<int> delays;
    std::uint8_t last_conversion_command{0};
};

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

bool nearly_equal(float left, float right, float tolerance)
{
    return std::fabs(left - right) <= tolerance;
}

} // namespace

int main()
{
    MockSPI spi;
    MS5607 baro(spi);

    require(baro.init(), "MS5607 init should succeed");
    require(
        !spi.commands.empty() && (spi.commands.front() == 0x1EU),
        "MS5607 should issue a reset command during init");

    baro_data sample{};
    require(baro.update(&sample), "MS5607 update should succeed after init");
    require(sample.pressure == 110002, "Unexpected compensated pressure");
    require(nearly_equal(sample.temperature, 20.0f, 0.01f), "Unexpected compensated temperature");
    require(nearly_equal(sample.altitude, 0.0f, 0.05f), "First compensated sample should be near the pressure baseline");

    return 0;
}
