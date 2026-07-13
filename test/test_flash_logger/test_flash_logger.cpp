#include <Flash/FlashLogger.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>

// Test the library implementation without pulling in hardware SPI dependencies.
#include "../../src/Flash/FlashLogger/FlashLogger.cpp"

#define CHECK_TRUE(expr) check_true((expr), #expr, __LINE__)
#define CHECK_FALSE(expr) check_false((expr), #expr, __LINE__)
#define CHECK_EQ(expected, actual) check_eq((expected), (actual), #actual, __LINE__)

static void fail(const char* expr, int line) {
    std::fprintf(stderr, "FAIL line %d: %s\n", line, expr);
    std::abort();
}

static void check_true(bool value, const char* expr, int line) {
    if (!value) {
        fail(expr, line);
    }
}

static void check_false(bool value, const char* expr, int line) {
    if (value) {
        fail(expr, line);
    }
}

template<typename T, typename U>
static void check_eq(T expected, U actual, const char* expr, int line) {
    if (!(expected == actual)) {
        std::fprintf(stderr, "expected=%lu actual=%lu\n",
                     static_cast<unsigned long>(expected),
                     static_cast<unsigned long>(actual));
        fail(expr, line);
    }
}

class FakeFlash : public Flash {
public:
    explicit FakeFlash(size_t size) : memory(size, FLASH_LOG_ERASED_BYTE) {}

    bool init() override {
        return true;
    }

    bool write(uint32_t address, const uint8_t* data, size_t length) override {
        if (data == nullptr || address > memory.size() || length > (memory.size() - address)) {
            return false;
        }

        write_count++;
        if (write_count == fail_on_write_count) {
            return false;
        }

        for (size_t i = 0U; i < length; i++) {
            memory[address + i] &= data[i];
        }

        return true;
    }

    bool read(uint32_t address, uint8_t* buffer, size_t length) override {
        if (buffer == nullptr || address > memory.size() || length > (memory.size() - address)) {
            return false;
        }

        std::memcpy(buffer, &memory[address], length);
        if (corrupt_verify_read && write_count >= 2U && length > 0U) {
            buffer[0] ^= 0x01U;
            corrupt_verify_read = false;
        }
        return true;
    }

    bool erase(uint32_t address, size_t length) override {
        if (address > memory.size() || length > (memory.size() - address)) {
            return false;
        }

        std::fill(memory.begin() + address,
                  memory.begin() + address + length,
                  FLASH_LOG_ERASED_BYTE);
        return true;
    }

    void clear_bits(uint32_t address, uint8_t mask) {
        memory[address] &= mask;
    }

    void fail_on_write(size_t write_number) {
        fail_on_write_count = write_number;
    }

    void allow_writes() {
        fail_on_write_count = std::numeric_limits<size_t>::max();
    }

    void corrupt_next_verify_read() {
        corrupt_verify_read = true;
    }

private:
    std::vector<uint8_t> memory;
    size_t write_count = 0U;
    size_t fail_on_write_count = std::numeric_limits<size_t>::max();
    bool corrupt_verify_read = false;
};

static FlashLogConfig test_config(uint32_t length) {
    FlashLogConfig config{};
    config.start_address = 0U;
    config.length = length;
    config.erase_block_size = 16U;
    config.write_alignment = 4U;
    config.max_payload_size = 64U;
    config.verify_writes = true;
    config.verify_existing_payloads_on_begin = true;
    return config;
}

static void test_begin_allocates_next_run_id_from_existing_records() {
    FakeFlash flash(512U);
    const FlashLogConfig config = test_config(512U);

    FlashLogger first(flash);
    CHECK_TRUE(first.begin(config));
    CHECK_EQ(1U, first.info().run_id);

    const uint8_t first_payload[] = {1U, 2U, 3U};
    CHECK_TRUE(first.append_typed(first_payload,
                                  sizeof(first_payload),
                                  100U,
                                  FlashLogPayloadType::FlightData));

    FlashLogger second(flash);
    CHECK_TRUE(second.begin(config));
    CHECK_EQ(2U, second.info().run_id);
    CHECK_EQ(1U, second.info().highest_run_id);
    CHECK_EQ(1U, second.info().record_count);

    const uint8_t second_payload[] = {4U, 5U};
    CHECK_TRUE(second.append_typed(second_payload,
                                   sizeof(second_payload),
                                   200U,
                                   FlashLogPayloadType::SecondaryFlightData));

    FlashLogCursor cursor = second.cursor();
    FlashLogRecordHeader header{};
    uint8_t payload[8] = {};
    size_t payload_length = 0U;

    CHECK_TRUE(second.read_next(cursor, header, payload, sizeof(payload), &payload_length));
    CHECK_EQ(1U, header.run_id);
    CHECK_EQ(0U, header.sequence);
    CHECK_EQ(static_cast<uint16_t>(FlashLogPayloadType::FlightData), header.payload_type);
    CHECK_EQ(FLASH_LOG_PAYLOAD_VERSION, header.payload_version);
    CHECK_EQ(sizeof(first_payload), payload_length);
    CHECK_TRUE(std::memcmp(first_payload, payload, sizeof(first_payload)) == 0);

    CHECK_TRUE(second.read_next(cursor, header, payload, sizeof(payload), &payload_length));
    CHECK_EQ(2U, header.run_id);
    CHECK_EQ(0U, header.sequence);
    CHECK_EQ(static_cast<uint16_t>(FlashLogPayloadType::SecondaryFlightData), header.payload_type);
    CHECK_EQ(FLASH_LOG_PAYLOAD_VERSION, header.payload_version);
    CHECK_EQ(sizeof(second_payload), payload_length);
    CHECK_TRUE(std::memcmp(second_payload, payload, sizeof(second_payload)) == 0);
}

static void test_full_log_does_not_wrap_or_overwrite() {
    FakeFlash flash(96U);
    FlashLogConfig config = test_config(96U);
    config.max_payload_size = 8U;

    FlashLogger logger(flash);
    CHECK_TRUE(logger.begin(config));

    const uint8_t payload[8] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U};
    CHECK_TRUE(logger.append(payload, sizeof(payload), 10U));
    CHECK_TRUE(logger.append(payload, sizeof(payload), 20U));
    CHECK_FALSE(logger.append(payload, sizeof(payload), 30U));
    CHECK_EQ(FlashLogStatus::Full, logger.status());
    CHECK_EQ(2U, logger.info().record_count);
}

static void test_exactly_full_log_reports_not_append_ready() {
    FakeFlash flash(96U);
    FlashLogConfig config = test_config(96U);
    config.erase_block_size = 4U;
    config.max_payload_size = 8U;

    FlashLogger logger(flash);
    CHECK_TRUE(logger.begin(config));

    const uint8_t payload[8] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U};
    CHECK_TRUE(logger.append(payload, sizeof(payload), 10U));
    CHECK_TRUE(logger.append(payload, sizeof(payload), 20U));
    CHECK_TRUE(logger.info().full);
    CHECK_FALSE(logger.info().append_ready);
    CHECK_FALSE(logger.append(payload, sizeof(payload), 30U));
    CHECK_EQ(FlashLogStatus::Full, logger.status());

    FlashLogger rebooted(flash);
    CHECK_FALSE(rebooted.begin(config));
    CHECK_TRUE(rebooted.info().full);
    CHECK_FALSE(rebooted.info().append_ready);
    CHECK_EQ(FlashLogStatus::Full, rebooted.status());
}

static void test_uncommitted_record_is_skipped_after_reboot() {
    FakeFlash flash(512U);
    const FlashLogConfig config = test_config(512U);

    FlashLogger writer(flash);
    CHECK_TRUE(writer.begin(config));

    const uint8_t interrupted_payload[] = {9U, 8U, 7U};
    flash.fail_on_write(3U);
    CHECK_FALSE(writer.append(interrupted_payload, sizeof(interrupted_payload), 10U));
    CHECK_EQ(FlashLogStatus::FlashError, writer.status());
    CHECK_FALSE(writer.append(interrupted_payload, sizeof(interrupted_payload), 11U));

    flash.allow_writes();

    FlashLogger recovered(flash);
    CHECK_TRUE(recovered.begin(config));
    CHECK_EQ(1U, recovered.info().run_id);
    CHECK_EQ(0U, recovered.info().record_count);

    const uint8_t good_payload[] = {1U, 2U, 3U, 4U};
    CHECK_TRUE(recovered.append(good_payload, sizeof(good_payload), 20U));

    FlashLogCursor cursor = recovered.cursor();
    FlashLogRecordHeader header{};
    uint8_t payload[8] = {};
    size_t payload_length = 0U;

    CHECK_TRUE(recovered.read_next(cursor, header, payload, sizeof(payload), &payload_length));
    CHECK_EQ(1U, header.run_id);
    CHECK_EQ(0U, header.sequence);
    CHECK_EQ(sizeof(good_payload), payload_length);
    CHECK_TRUE(std::memcmp(good_payload, payload, sizeof(good_payload)) == 0);
    CHECK_FALSE(recovered.read_next(cursor, header, payload, sizeof(payload), &payload_length));
    CHECK_EQ(FlashLogStatus::NotFound, recovered.status());
}

static void test_payload_corruption_blocks_append_until_explicit_erase() {
    FakeFlash flash(256U);
    const FlashLogConfig config = test_config(256U);

    FlashLogger writer(flash);
    CHECK_TRUE(writer.begin(config));

    uint32_t record_address = 0U;
    const uint8_t payload[] = {0xFFU, 0xAAU, 0x55U};
    CHECK_TRUE(writer.append(payload, sizeof(payload), 123U, &record_address));

    flash.clear_bits(record_address + sizeof(FlashLogRecordHeader), 0xFEU);

    FlashLogger reader(flash);
    CHECK_FALSE(reader.begin(config));
    CHECK_EQ(FlashLogStatus::Corrupt, reader.status());
    CHECK_FALSE(reader.append(payload, sizeof(payload), 124U));

    CHECK_TRUE(reader.erase_all());
    CHECK_TRUE(reader.append(payload, sizeof(payload), 125U));
    CHECK_EQ(1U, reader.info().record_count);
}

static void test_payload_verification_can_be_deferred_during_begin() {
    FakeFlash flash(256U);
    FlashLogConfig config = test_config(256U);

    FlashLogger writer(flash);
    CHECK_TRUE(writer.begin(config));

    uint32_t record_address = 0U;
    const uint8_t first_payload[] = {0xFFU, 0xAAU, 0x55U};
    CHECK_TRUE(writer.append(first_payload, sizeof(first_payload), 123U, &record_address));
    flash.clear_bits(record_address + sizeof(FlashLogRecordHeader), 0xFEU);

    config.verify_existing_payloads_on_begin = false;
    FlashLogger recovered(flash);
    CHECK_TRUE(recovered.begin(config));
    CHECK_EQ(1U, recovered.info().record_count);
    CHECK_EQ(2U, recovered.info().run_id);

    const uint8_t second_payload[] = {1U, 2U, 3U, 4U};
    CHECK_TRUE(recovered.append(second_payload, sizeof(second_payload), 124U));
    CHECK_EQ(2U, recovered.info().record_count);
}

static void test_transient_verify_read_is_retried() {
    FakeFlash flash(256U);
    const FlashLogConfig config = test_config(256U);

    FlashLogger logger(flash);
    CHECK_TRUE(logger.begin(config));

    const uint8_t payload[] = {1U, 2U, 3U, 4U};
    flash.corrupt_next_verify_read();
    CHECK_TRUE(logger.append(payload, sizeof(payload), 123U));
    CHECK_EQ(FlashLogStatus::Ok, logger.status());
    CHECK_EQ(1U, logger.info().record_count);
}

int main() {
    test_begin_allocates_next_run_id_from_existing_records();
    test_full_log_does_not_wrap_or_overwrite();
    test_exactly_full_log_reports_not_append_ready();
    test_uncommitted_record_is_skipped_after_reboot();
    test_payload_corruption_blocks_append_until_explicit_erase();
    test_payload_verification_can_be_deferred_during_begin();
    test_transient_verify_read_is_retried();
    std::puts("FlashLogger tests passed");
    return 0;
}
