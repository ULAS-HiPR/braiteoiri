#pragma once

#include <Radio/Radio.h>
#include <SPI/SPI_Handler.h>

#include <cstddef>
#include <cstdint>

enum class sx1272_bandwidth_t : std::uint8_t {
    BW_125_KHZ = 0,
    BW_250_KHZ = 1,
    BW_500_KHZ = 2,
};

enum class sx1272_coding_rate_t : std::uint8_t {
    CR_4_5 = 1,
    CR_4_6 = 2,
    CR_4_7 = 3,
    CR_4_8 = 4,
};

using sx1272_reset_write_t = void (*)(bool high, void* context);
using sx1272_delay_ms_t = void (*)(std::uint32_t ms, void* context);

enum class sx1272_switch_mode_t : std::uint8_t {
    OFF = 0,
    RX = 1,
    TX = 2,
};

using sx1272_switch_write_t = void (*)(sx1272_switch_mode_t mode, void* context);

struct sx1272_pins_t {
    sx1272_reset_write_t reset_write{nullptr};
    void* reset_context{nullptr};
    sx1272_delay_ms_t delay_ms{nullptr};
    void* delay_context{nullptr};
    sx1272_switch_write_t switch_write{nullptr};
    void* switch_context{nullptr};
    bool reset_active_high{true};
};

struct sx1272_config_t {
    std::uint32_t frequency_hz{868000000U};
    std::int8_t tx_power_dbm{14};
    std::uint8_t spreading_factor{7};
    sx1272_bandwidth_t bandwidth{sx1272_bandwidth_t::BW_125_KHZ};
    sx1272_coding_rate_t coding_rate{sx1272_coding_rate_t::CR_4_5};
    std::uint16_t preamble_symbols{8};
    bool crc_on{true};
    bool implicit_header{false};
    std::uint8_t sync_word{0x12};
};

class SX1272 : public Radio {
public:
    SX1272(SPI_Handler& spi_handler, int cs_id, sx1272_pins_t pins);

    bool init() override;
    bool init(const sx1272_config_t& config);
    bool send(const std::uint8_t* data, std::size_t len) override;
    bool receive(std::uint8_t* data, std::size_t max_len, std::size_t* len) override;
    bool start_receive();
    bool tx_done();

    std::uint8_t version();
    std::uint8_t irq_flags();
    void clear_irq(std::uint8_t flags);
    std::int16_t packet_rssi_dbm();
    std::int8_t packet_snr_db();
    std::uint8_t last_error() const;

private:
    void reset();
    void delay_ms(std::uint32_t ms);
    bool write_register(std::uint8_t reg, std::uint8_t value);
    bool read_register(std::uint8_t reg, std::uint8_t* value);
    bool write_fifo(const std::uint8_t* data, std::size_t len);
    bool read_fifo(std::uint8_t* data, std::size_t len);
    bool set_mode(std::uint8_t mode);
    bool set_lora_sleep();
    bool set_frequency(std::uint32_t frequency_hz);
    bool set_tx_power(std::int8_t dbm);
    bool configure_lora(const sx1272_config_t& config);
    void set_switch(sx1272_switch_mode_t mode);

    SPI_Handler& spi;
    int cs;
    sx1272_pins_t pins;
    sx1272_config_t active_config{};
    std::uint8_t error{0U};
};
