#include <GNSS/MAXM10S.h>

#include <cstdlib>
#include <cstring>

namespace {

int hex_value(char c)
{
    if ((c >= '0') && (c <= '9')) {
        return c - '0';
    }
    if ((c >= 'A') && (c <= 'F')) {
        return 10 + (c - 'A');
    }
    if ((c >= 'a') && (c <= 'f')) {
        return 10 + (c - 'a');
    }
    return -1;
}

} // namespace

MAXM10S::MAXM10S(UART_Handler& uart_handler)
    : uart(uart_handler)
{
}

bool MAXM10S::init()
{
    line_len = 0U;
    valid_fix = false;
    fresh_fix = false;
    seen = 0U;
    checksum_bad = 0U;
    parsed = 0U;
    current = gps_data{};
    return true;
}

bool MAXM10S::update(gps_data* data)
{
    service();

    if (data != nullptr) {
        *data = current;
    }

    bool updated = fresh_fix;
    fresh_fix = false;
    return updated;
}

bool MAXM10S::service(std::size_t max_bytes)
{
    bool updated = false;

    for (std::size_t i = 0U; i < max_bytes; ++i) {
        std::uint8_t byte = 0U;
        if (uart.read(&byte, 1U, 0U) != 1U) {
            break;
        }
        updated = process_byte(static_cast<char>(byte)) || updated;
    }

    return updated;
}

bool MAXM10S::fix_valid() const
{
    return valid_fix;
}

std::uint32_t MAXM10S::messages_seen() const
{
    return seen;
}

std::uint32_t MAXM10S::checksum_failures() const
{
    return checksum_bad;
}

std::uint32_t MAXM10S::sentences_parsed() const
{
    return parsed;
}

const gps_data& MAXM10S::last_data() const
{
    return current;
}

bool MAXM10S::process_byte(char byte)
{
    if (byte == '$') {
        line[0] = byte;
        line_len = 1U;
        return false;
    }

    if (line_len == 0U) {
        return false;
    }

    if ((byte == '\r') || (byte == '\n')) {
        line[line_len] = '\0';
        bool parsed_sentence = parse_sentence(line);
        line_len = 0U;
        return parsed_sentence;
    }

    if (line_len >= (sizeof(line) - 1U)) {
        line_len = 0U;
        return false;
    }

    line[line_len++] = byte;
    return false;
}

bool MAXM10S::parse_sentence(const char* sentence)
{
    if ((sentence == nullptr) || (sentence[0] != '$')) {
        return false;
    }

    if (!checksum_ok(sentence)) {
        checksum_bad++;
        return false;
    }

    seen++;

    char work[sizeof(line)]{};
    std::strncpy(work, sentence, sizeof(work) - 1U);

    char* checksum = std::strchr(work, '*');
    if (checksum != nullptr) {
        *checksum = '\0';
    }

    char* fields[24]{};
    std::size_t count = 0U;
    if (!split_fields(&work[1], fields, 24U, &count) || (count == 0U)) {
        return false;
    }

    bool ok = false;
    if (message_is(fields[0], "GGA")) {
        ok = parse_gga(fields, count);
    } else if (message_is(fields[0], "RMC")) {
        ok = parse_rmc(fields, count);
    }

    if (ok) {
        parsed++;
    }

    return ok;
}

bool MAXM10S::parse_gga(char** fields, std::size_t count)
{
    if (count < 10U) {
        return false;
    }

    std::uint8_t quality = 0U;
    if (!parse_uint8(fields[6], &quality)) {
        valid_fix = false;
        fresh_fix = false;
        return false;
    }

    std::uint8_t sats = 0U;
    (void)parse_uint8(fields[7], &sats);
    current.satellites = sats;

    valid_fix = quality > 0U;
    if (!valid_fix) {
        fresh_fix = false;
        return false;
    }

    double altitude = 0.0;
    if (!parse_float(fields[9], &altitude)) {
        fresh_fix = false;
        return false;
    }

    double latitude = 0.0;
    double longitude = 0.0;
    if (!parse_lat_lon(fields[2], fields[3], &latitude) ||
        !parse_lat_lon(fields[4], fields[5], &longitude)) {
        valid_fix = false;
        fresh_fix = false;
        return false;
    }

    current.altitude = static_cast<float>(altitude);
    current.latitude = latitude;
    current.longitude = longitude;
    fresh_fix = true;
    return true;
}

bool MAXM10S::parse_rmc(char** fields, std::size_t count)
{
    if (count < 8U) {
        return false;
    }

    valid_fix = (fields[2] != nullptr) && (fields[2][0] == 'A');
    if (!valid_fix) {
        fresh_fix = false;
        return false;
    }

    double latitude = 0.0;
    double longitude = 0.0;
    if (!parse_lat_lon(fields[3], fields[4], &latitude) ||
        !parse_lat_lon(fields[5], fields[6], &longitude)) {
        valid_fix = false;
        fresh_fix = false;
        return false;
    }

    double speed_knots = 0.0;
    if (!parse_float(fields[7], &speed_knots)) {
        fresh_fix = false;
        return false;
    }

    current.latitude = latitude;
    current.longitude = longitude;
    current.velocity = static_cast<float>(speed_knots * 0.514444);
    fresh_fix = true;
    return true;
}

bool MAXM10S::checksum_ok(const char* sentence) const
{
    const char* checksum = std::strchr(sentence, '*');
    if ((checksum == nullptr) || (checksum[1] == '\0') || (checksum[2] == '\0')) {
        return false;
    }

    std::uint8_t calculated = 0U;
    for (const char* cursor = sentence + 1; cursor < checksum; ++cursor) {
        calculated ^= static_cast<std::uint8_t>(*cursor);
    }

    int high = hex_value(checksum[1]);
    int low = hex_value(checksum[2]);
    if ((high < 0) || (low < 0)) {
        return false;
    }

    std::uint8_t provided = static_cast<std::uint8_t>((high << 4) | low);
    return calculated == provided;
}

bool MAXM10S::split_fields(char* sentence, char** fields, std::size_t max_fields, std::size_t* count) const
{
    if ((sentence == nullptr) || (fields == nullptr) || (count == nullptr) || (max_fields == 0U)) {
        return false;
    }

    std::size_t found = 0U;
    fields[found++] = sentence;

    for (char* cursor = sentence; *cursor != '\0'; ++cursor) {
        if (*cursor == ',') {
            *cursor = '\0';
            if (found >= max_fields) {
                return false;
            }
            fields[found++] = cursor + 1;
        }
    }

    *count = found;
    return true;
}

bool MAXM10S::message_is(const char* talker_type, const char* type) const
{
    if ((talker_type == nullptr) || (type == nullptr)) {
        return false;
    }

    std::size_t len = std::strlen(talker_type);
    if (len < 3U) {
        return false;
    }

    return std::strcmp(&talker_type[len - 3U], type) == 0;
}

bool MAXM10S::parse_lat_lon(const char* value, const char* hemisphere, double* out) const
{
    if ((value == nullptr) || (hemisphere == nullptr) || (out == nullptr) || (value[0] == '\0')) {
        return false;
    }

    double raw = 0.0;
    if (!parse_float(value, &raw)) {
        return false;
    }

    int degrees = static_cast<int>(raw / 100.0);
    double minutes = raw - (static_cast<double>(degrees) * 100.0);
    if ((degrees < 0) || (minutes < 0.0) || (minutes >= 60.0)) {
        return false;
    }

    double result = static_cast<double>(degrees) + (minutes / 60.0);

    if ((hemisphere[0] == 'S') || (hemisphere[0] == 'W')) {
        result = -result;
    } else if ((hemisphere[0] != 'N') && (hemisphere[0] != 'E')) {
        return false;
    }

    if (((hemisphere[0] == 'N') || (hemisphere[0] == 'S')) && (result > 90.0 || result < -90.0)) {
        return false;
    }

    if (((hemisphere[0] == 'E') || (hemisphere[0] == 'W')) && (result > 180.0 || result < -180.0)) {
        return false;
    }

    *out = result;
    return true;
}

bool MAXM10S::parse_float(const char* value, double* out) const
{
    if ((value == nullptr) || (out == nullptr) || (value[0] == '\0')) {
        return false;
    }

    char* end = nullptr;
    double parsed_value = std::strtod(value, &end);
    if ((end == value) || (*end != '\0')) {
        return false;
    }

    *out = parsed_value;
    return true;
}

bool MAXM10S::parse_uint8(const char* value, std::uint8_t* out) const
{
    if ((value == nullptr) || (out == nullptr) || (value[0] == '\0')) {
        return false;
    }

    char* end = nullptr;
    long parsed_value = std::strtol(value, &end, 10);
    if ((end == value) || (parsed_value < 0L) || (parsed_value > 255L)) {
        return false;
    }

    *out = static_cast<std::uint8_t>(parsed_value);
    return true;
}
