#include <Radio/SX1272.h>

namespace {

constexpr std::uint8_t REG_FIFO = 0x00;
constexpr std::uint8_t REG_OP_MODE = 0x01;
constexpr std::uint8_t REG_FRF_MSB = 0x06;
constexpr std::uint8_t REG_PA_CONFIG = 0x09;
constexpr std::uint8_t REG_LNA = 0x0C;
constexpr std::uint8_t REG_FIFO_ADDR_PTR = 0x0D;
constexpr std::uint8_t REG_FIFO_TX_BASE_ADDR = 0x0E;
constexpr std::uint8_t REG_FIFO_RX_BASE_ADDR = 0x0F;
constexpr std::uint8_t REG_FIFO_RX_CURRENT_ADDR = 0x10;
constexpr std::uint8_t REG_IRQ_FLAGS = 0x12;
constexpr std::uint8_t REG_RX_NB_BYTES = 0x13;
constexpr std::uint8_t REG_PKT_SNR_VALUE = 0x19;
constexpr std::uint8_t REG_PKT_RSSI_VALUE = 0x1A;
constexpr std::uint8_t REG_MODEM_CONFIG_1 = 0x1D;
constexpr std::uint8_t REG_MODEM_CONFIG_2 = 0x1E;
constexpr std::uint8_t REG_PREAMBLE_MSB = 0x20;
constexpr std::uint8_t REG_PAYLOAD_LENGTH = 0x22;
constexpr std::uint8_t REG_SYNC_WORD = 0x39;
constexpr std::uint8_t REG_DIO_MAPPING_1 = 0x40;
constexpr std::uint8_t REG_VERSION = 0x42;

constexpr std::uint8_t MODE_SLEEP = 0x00;
constexpr std::uint8_t MODE_STDBY = 0x01;
constexpr std::uint8_t MODE_TX = 0x03;
constexpr std::uint8_t MODE_RX_CONTINUOUS = 0x05;

constexpr std::uint8_t LONG_RANGE_MODE = 0x80;
constexpr std::uint8_t IRQ_RX_DONE = 0x40;
constexpr std::uint8_t IRQ_PAYLOAD_CRC_ERROR = 0x20;
constexpr std::uint8_t IRQ_TX_DONE = 0x08;
constexpr std::uint8_t IRQ_ALL = 0xFF;
constexpr std::uint8_t EXPECTED_VERSION = 0x22;

constexpr std::uint8_t ERROR_NONE = 0U;
constexpr std::uint8_t ERROR_BAD_VERSION = 1U;
constexpr std::uint8_t ERROR_BAD_SEND_ARG = 2U;
constexpr std::uint8_t ERROR_BAD_RECEIVE_ARG = 3U;
constexpr std::uint8_t ERROR_RX_CRC = 4U;
constexpr std::uint8_t ERROR_RX_TOO_LARGE = 5U;
constexpr std::uint8_t ERROR_WRITE_REGISTER = 6U;
constexpr std::uint8_t ERROR_READ_REGISTER_ARG = 7U;
constexpr std::uint8_t ERROR_READ_REGISTER = 8U;
constexpr std::uint8_t ERROR_WRITE_FIFO = 9U;
constexpr std::uint8_t ERROR_READ_FIFO = 10U;
constexpr std::uint8_t ERROR_BAD_CONFIG = 11U;

std::uint8_t clamp_power(std::int8_t dbm)
{
    if (dbm < 2) {
        return 0U;
    }
    if (dbm > 17) {
        return 15U;
    }
    return static_cast<std::uint8_t>(dbm - 2);
}

} // namespace

SX1272::SX1272(SPI_Handler& spi_handler, int cs_id, sx1272_pins_t gpio_pins)
    : spi(spi_handler), cs(cs_id), pins(gpio_pins)
{
}

bool SX1272::init()
{
    return init(active_config);
}

bool SX1272::init(const sx1272_config_t& config)
{
    error = ERROR_NONE;
    active_config = config;

    reset();
    set_switch(sx1272_switch_mode_t::OFF);

    std::uint8_t chip_version = version();
    if (chip_version != EXPECTED_VERSION) {
        error = ERROR_BAD_VERSION;
        return false;
    }

    if (!set_lora_sleep()) {
        return false;
    }

    return configure_lora(active_config);
}

bool SX1272::send(const std::uint8_t* data, std::size_t len)
{
    error = ERROR_NONE;
    if ((data == nullptr) || (len == 0U) || (len > 255U)) {
        error = ERROR_BAD_SEND_ARG;
        return false;
    }

    if (!set_mode(MODE_STDBY)) {
        return false;
    }

    clear_irq(IRQ_ALL);
    if (!write_register(REG_FIFO_TX_BASE_ADDR, 0U) ||
        !write_register(REG_FIFO_ADDR_PTR, 0U) ||
        !write_register(REG_PAYLOAD_LENGTH, static_cast<std::uint8_t>(len)) ||
        !write_fifo(data, len)) {
        return false;
    }

    set_switch(sx1272_switch_mode_t::TX);
    if (!set_mode(MODE_TX)) {
        set_switch(sx1272_switch_mode_t::OFF);
        return false;
    }

    return true;
}

bool SX1272::receive(std::uint8_t* data, std::size_t max_len, std::size_t* len)
{
    error = ERROR_NONE;
    if ((data == nullptr) || (len == nullptr)) {
        error = ERROR_BAD_RECEIVE_ARG;
        return false;
    }

    *len = 0U;

    std::uint8_t flags = irq_flags();
    if ((flags & IRQ_PAYLOAD_CRC_ERROR) != 0U) {
        clear_irq(static_cast<std::uint8_t>(IRQ_PAYLOAD_CRC_ERROR | IRQ_RX_DONE));
        error = ERROR_RX_CRC;
        return false;
    }

    if ((flags & IRQ_RX_DONE) == 0U) {
        return false;
    }

    std::uint8_t received = 0U;
    std::uint8_t current_addr = 0U;
    if (!read_register(REG_RX_NB_BYTES, &received) ||
        !read_register(REG_FIFO_RX_CURRENT_ADDR, &current_addr)) {
        return false;
    }

    if (received > max_len) {
        clear_irq(static_cast<std::uint8_t>(IRQ_RX_DONE | IRQ_PAYLOAD_CRC_ERROR));
        error = ERROR_RX_TOO_LARGE;
        return false;
    }

    if (!write_register(REG_FIFO_ADDR_PTR, current_addr) ||
        !read_fifo(data, received)) {
        return false;
    }

    clear_irq(IRQ_RX_DONE);
    *len = received;
    return true;
}

bool SX1272::start_receive()
{
    clear_irq(IRQ_ALL);
    set_switch(sx1272_switch_mode_t::RX);
    if (!set_mode(MODE_RX_CONTINUOUS)) {
        set_switch(sx1272_switch_mode_t::OFF);
        return false;
    }

    return true;
}

bool SX1272::tx_done()
{
    std::uint8_t flags = irq_flags();
    if ((flags & IRQ_TX_DONE) == 0U) {
        return false;
    }

    clear_irq(IRQ_TX_DONE);
    set_switch(sx1272_switch_mode_t::OFF);
    return true;
}

std::uint8_t SX1272::version()
{
    std::uint8_t value = 0U;
    if (!read_register(REG_VERSION, &value)) {
        return 0U;
    }
    return value;
}

std::uint8_t SX1272::irq_flags()
{
    std::uint8_t value = 0U;
    (void)read_register(REG_IRQ_FLAGS, &value);
    return value;
}

void SX1272::clear_irq(std::uint8_t flags)
{
    (void)write_register(REG_IRQ_FLAGS, flags);
}

std::int16_t SX1272::packet_rssi_dbm()
{
    std::uint8_t value = 0U;
    if (!read_register(REG_PKT_RSSI_VALUE, &value)) {
        return 0;
    }
    return static_cast<std::int16_t>(value) - 139;
}

std::int8_t SX1272::packet_snr_db()
{
    std::uint8_t value = 0U;
    if (!read_register(REG_PKT_SNR_VALUE, &value)) {
        return 0;
    }
    return static_cast<std::int8_t>(value) / 4;
}

std::uint8_t SX1272::last_error() const
{
    return error;
}

void SX1272::reset()
{
    if (pins.reset_write == nullptr) {
        return;
    }

    bool active = pins.reset_active_high;
    pins.reset_write(active, pins.reset_context);
    delay_ms(10U);
    pins.reset_write(!active, pins.reset_context);
    delay_ms(10U);
}

void SX1272::delay_ms(std::uint32_t ms)
{
    if (pins.delay_ms != nullptr) {
        pins.delay_ms(ms, pins.delay_context);
    } else {
        spi.delay_ms(static_cast<int>(ms));
    }
}

void SX1272::set_switch(sx1272_switch_mode_t mode)
{
    if (pins.switch_write != nullptr) {
        pins.switch_write(mode, pins.switch_context);
    }
}

bool SX1272::write_register(std::uint8_t reg, std::uint8_t value)
{
    std::uint8_t tx[2] = { static_cast<std::uint8_t>(reg | 0x80U), value };
    std::uint8_t rx[2] = {};

    spi.cs_select(cs);
    bool ok = spi.transfer(tx, rx, sizeof(tx));
    spi.cs_deselect(cs);

    if (!ok) {
        error = ERROR_WRITE_REGISTER;
    }
    return ok;
}

bool SX1272::read_register(std::uint8_t reg, std::uint8_t* value)
{
    if (value == nullptr) {
        error = ERROR_READ_REGISTER_ARG;
        return false;
    }

    std::uint8_t tx[2] = { static_cast<std::uint8_t>(reg & 0x7FU), 0U };
    std::uint8_t rx[2] = {};

    spi.cs_select(cs);
    bool ok = spi.transfer(tx, rx, sizeof(tx));
    spi.cs_deselect(cs);

    if (!ok) {
        error = ERROR_READ_REGISTER;
        return false;
    }

    *value = rx[1];
    return true;
}

bool SX1272::write_fifo(const std::uint8_t* data, std::size_t len)
{
    if ((data == nullptr) && (len > 0U)) {
        error = ERROR_BAD_SEND_ARG;
        return false;
    }

    std::uint8_t addr = static_cast<std::uint8_t>(REG_FIFO | 0x80U);

    spi.cs_select(cs);
    bool ok = spi.transmit(&addr, 1U) && spi.transmit(data, len);
    spi.cs_deselect(cs);

    if (!ok) {
        error = ERROR_WRITE_FIFO;
    }
    return ok;
}

bool SX1272::read_fifo(std::uint8_t* data, std::size_t len)
{
    if ((data == nullptr) && (len > 0U)) {
        error = ERROR_BAD_RECEIVE_ARG;
        return false;
    }

    std::uint8_t addr = REG_FIFO;

    spi.cs_select(cs);
    bool ok = spi.transmit(&addr, 1U) && spi.receive(data, len);
    spi.cs_deselect(cs);

    if (!ok) {
        error = ERROR_READ_FIFO;
    }
    return ok;
}

bool SX1272::set_mode(std::uint8_t mode)
{
    std::uint8_t op_mode = 0U;
    if (!read_register(REG_OP_MODE, &op_mode)) {
        return false;
    }

    op_mode = static_cast<std::uint8_t>((op_mode & 0xF8U) | LONG_RANGE_MODE | (mode & 0x07U));
    return write_register(REG_OP_MODE, op_mode);
}

bool SX1272::set_lora_sleep()
{
    bool ok = write_register(REG_OP_MODE, LONG_RANGE_MODE | MODE_SLEEP);
    delay_ms(10U);
    return ok;
}

bool SX1272::set_frequency(std::uint32_t frequency_hz)
{
    std::uint64_t frf = (static_cast<std::uint64_t>(frequency_hz) << 19U) / 32000000ULL;

    return write_register(REG_FRF_MSB, static_cast<std::uint8_t>((frf >> 16U) & 0xFFU)) &&
           write_register(static_cast<std::uint8_t>(REG_FRF_MSB + 1U), static_cast<std::uint8_t>((frf >> 8U) & 0xFFU)) &&
           write_register(static_cast<std::uint8_t>(REG_FRF_MSB + 2U), static_cast<std::uint8_t>(frf & 0xFFU));
}

bool SX1272::set_tx_power(std::int8_t dbm)
{
    return write_register(REG_PA_CONFIG, static_cast<std::uint8_t>(0x80U | clamp_power(dbm)));
}

bool SX1272::configure_lora(const sx1272_config_t& config)
{
    if ((config.frequency_hz < 860000000U) ||
        (config.frequency_hz > 1020000000U) ||
        (config.spreading_factor < 7U) ||
        (config.spreading_factor > 12U) ||
        (static_cast<std::uint8_t>(config.bandwidth) > static_cast<std::uint8_t>(sx1272_bandwidth_t::BW_500_KHZ)) ||
        (static_cast<std::uint8_t>(config.coding_rate) < static_cast<std::uint8_t>(sx1272_coding_rate_t::CR_4_5)) ||
        (static_cast<std::uint8_t>(config.coding_rate) > static_cast<std::uint8_t>(sx1272_coding_rate_t::CR_4_8)) ||
        (config.preamble_symbols == 0U)) {
        error = ERROR_BAD_CONFIG;
        return false;
    }

    bool low_data_rate = (config.spreading_factor >= 11U) &&
        (config.bandwidth == sx1272_bandwidth_t::BW_125_KHZ);

    std::uint8_t modem_config_1 =
        (static_cast<std::uint8_t>(config.bandwidth) << 6U) |
        (static_cast<std::uint8_t>(config.coding_rate) << 3U) |
        (config.implicit_header ? 0x04U : 0U) |
        (config.crc_on ? 0x02U : 0U) |
        (low_data_rate ? 0x01U : 0U);

    std::uint8_t modem_config_2 =
        static_cast<std::uint8_t>((config.spreading_factor << 4U) | 0x04U);

    return set_mode(MODE_STDBY) &&
           set_frequency(config.frequency_hz) &&
           write_register(REG_FIFO_TX_BASE_ADDR, 0U) &&
           write_register(REG_FIFO_RX_BASE_ADDR, 0U) &&
           write_register(REG_LNA, 0x23U) &&
           write_register(REG_MODEM_CONFIG_1, modem_config_1) &&
           write_register(REG_MODEM_CONFIG_2, modem_config_2) &&
           write_register(REG_PREAMBLE_MSB, static_cast<std::uint8_t>((config.preamble_symbols >> 8U) & 0xFFU)) &&
           write_register(static_cast<std::uint8_t>(REG_PREAMBLE_MSB + 1U), static_cast<std::uint8_t>(config.preamble_symbols & 0xFFU)) &&
           write_register(REG_SYNC_WORD, config.sync_word) &&
           write_register(REG_DIO_MAPPING_1, 0x00U) &&
           set_tx_power(config.tx_power_dbm) &&
           write_register(REG_PAYLOAD_LENGTH, 0U) &&
           set_mode(MODE_STDBY);
}
