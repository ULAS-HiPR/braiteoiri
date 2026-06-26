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
    bool poll_navigation_satellites() override;

    bool fix_valid() const;
    std::uint32_t bytes_seen() const;
    std::uint32_t messages_seen() const;
    std::uint32_t checksum_failures() const;
    std::uint32_t sentences_parsed() const;
    std::uint32_t sentences_started() const;
    std::uint32_t line_overflows() const;
    std::uint32_t text_messages_seen() const;
    std::uint32_t navigation_satellite_messages_seen() const;
    std::uint8_t satellites_in_view() const;
    std::uint8_t navigation_satellites_reported() const;
    std::uint8_t navigation_satellites_with_signal() const;
    std::uint8_t navigation_satellite_max_cno() const;
    std::uint8_t antenna_status() const;
    std::uint8_t last_byte() const;
    const gps_data& last_data() const;

private:
    bool process_byte(char byte);
    bool process_ubx_byte(std::uint8_t byte);
    bool parse_ubx_message();
    void reset_ubx_parser();
    void update_ubx_checksum(std::uint8_t byte);
    bool parse_sentence(const char* sentence);
    bool parse_gga(char** fields, std::size_t count);
    bool parse_gns(char** fields, std::size_t count);
    bool parse_gsv(char** fields, std::size_t count);
    bool parse_rmc(char** fields, std::size_t count);
    bool parse_txt(char** fields, std::size_t count);
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
    std::uint32_t raw_bytes{0};
    std::uint32_t seen{0};
    std::uint32_t checksum_bad{0};
    std::uint32_t parsed{0};
    std::uint32_t starts{0};
    std::uint32_t overflows{0};
    std::uint32_t txt_seen{0};
    std::uint32_t nav_sat_seen{0};
    std::uint8_t sats_in_view{0};
    std::uint8_t nav_sat_reported{0};
    std::uint8_t nav_sat_signal{0};
    std::uint8_t nav_sat_max_cno{0};
    std::uint8_t ant_status{0};
    std::uint8_t last_rx_byte{0};
    std::uint8_t ubx_state{0};
    std::uint8_t ubx_class{0};
    std::uint8_t ubx_id{0};
    std::uint16_t ubx_len{0};
    std::uint16_t ubx_index{0};
    std::uint8_t ubx_ck_a{0};
    std::uint8_t ubx_ck_b{0};
    std::uint8_t ubx_payload[512]{};
};