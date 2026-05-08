#ifndef MS5607_H
#define MS5607_H

#include <Baro/baro.h>
#include <SPI/SPI_Handler.h>

#include <array>
#include <cstdint>

enum ms5607_osr_t : uint8_t {
    MS5607_OSR_256 = 0x00,
    MS5607_OSR_512 = 0x02,
    MS5607_OSR_1024 = 0x04,
    MS5607_OSR_2048 = 0x06,
    MS5607_OSR_4096 = 0x08,
};

class MS5607 : public Baro {
public:
    explicit MS5607(SPI_Handler& spi_handler)
        : spi_handler(spi_handler) {}

    bool init() override;
    bool update(baro_data* data) override;

private:
    void send_command(std::uint8_t command);
    void read_command(std::uint8_t command, std::uint8_t* data, std::uint16_t len);
    bool read_prom();
    std::uint16_t read_prom_word(std::uint8_t index);
    std::uint8_t calculate_crc4() const;
    std::uint32_t convert_and_read(std::uint8_t command, int delay_ms);
    bool read_compensated_sample(std::int32_t* pressure_pa, float* temperature_c);
    float pressure_to_altitude(float pressure_pa) const;

    SPI_Handler& spi_handler;
    std::array<std::uint16_t, 8> prom{};
    ms5607_osr_t osr{MS5607_OSR_4096};
    float reference_pressure_pa{101325.0f};
    bool reference_pressure_valid{false};
    bool initialized{false};
};

#endif // MS5607_H
