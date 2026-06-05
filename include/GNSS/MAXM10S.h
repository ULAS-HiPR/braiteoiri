#pragma once

#include <GNSS/GNSS.h>
#include <UART/UART_Handler.h>

#include <cstddef>
#include <cstdint>

class MAXM10S : public GNSS {
public:
    explicit MAXM10S(UART_Handler& uart_handler);

    bool init() override;
    bool update(gps_data* data) override;
    bool service(std::size_t max_bytes = 256U);

    bool fix_valid() const;
    std::uint32_t messages_seen() const;
    std::uint32_t checksum_failures() const;
    std::uint32_t sentences_parsed() const;
    const gps_data& last_data() const;

private:
    bool process_byte(char byte);
    bool parse_sentence(const char* sentence);
    bool parse_gga(char** fields, std::size_t count);
    bool parse_rmc(char** fields, std::size_t count);
    bool checksum_ok(const char* sentence) const;
    bool split_fields(char* sentence, char** fields, std::size_t max_fields, std::size_t* count) const;
    bool message_is(const char* talker_type, const char* type) const;
    bool parse_lat_lon(const char* value, const char* hemisphere, double* out) const;
    bool parse_float(const char* value, double* out) const;
    bool parse_uint8(const char* value, std::uint8_t* out) const;

    UART_Handler& uart;
    gps_data current{};
    char line[128]{};
    std::size_t line_len{0};
    bool valid_fix{false};
    bool fresh_fix{false};
    std::uint32_t seen{0};
    std::uint32_t checksum_bad{0};
    std::uint32_t parsed{0};
};
