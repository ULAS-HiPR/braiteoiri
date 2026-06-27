#include <Flash/FlashLogger.h>

#include <cstddef>
#include <cstring>

namespace {
constexpr uint32_t FNV1A_OFFSET = 2166136261U;
constexpr uint32_t FNV1A_PRIME = 16777619U;
constexpr uint32_t MAX_DEFAULT_PAYLOAD_SIZE = 1024U;
constexpr uint32_t MX25_DEFAULT_SECTOR_SIZE = 0x1000U;
constexpr uint32_t BLANK_CHECK_CHUNK = 64U;
constexpr uint32_t VERIFY_CHUNK = 64U;

bool range_overflows(uint32_t start, uint32_t length) {
    return length == 0U || start > (UINT32_MAX - length);
}
}

FlashLogger::FlashLogger(Flash& flash) : flash_(flash) {}

FlashLogConfig FlashLogger::default_mx25_config(uint32_t start_address,
                                                 uint32_t length) {
    FlashLogConfig config{};
    config.start_address = start_address;
    config.length = length;
    config.erase_block_size = MX25_DEFAULT_SECTOR_SIZE;
    config.write_alignment = 4U;
    config.max_payload_size = MAX_DEFAULT_PAYLOAD_SIZE;
    config.verify_writes = true;
    return config;
}

bool FlashLogger::begin(uint32_t start_address, uint32_t length) {
    return begin(default_mx25_config(start_address, length));
}

bool FlashLogger::begin(uint32_t start_address, uint32_t length,
                        uint32_t run_id) {
    return begin(default_mx25_config(start_address, length), run_id);
}

bool FlashLogger::begin(const FlashLogConfig& config) {
    return begin_with_mode(config, 0U, true);
}

bool FlashLogger::begin(const FlashLogConfig& config, uint32_t run_id) {
    return begin_with_mode(config, run_id, false);
}

bool FlashLogger::begin_with_mode(const FlashLogConfig& config,
                                  uint32_t run_id,
                                  bool allocate_run_id) {
    initialized_ = false;
    append_ready_ = false;
    full_ = false;
    next_address_ = 0U;
    record_count_ = 0U;
    run_record_count_ = 0U;
    run_id_ = run_id;
    highest_run_id_ = 0U;
    config_ = config;

    if (range_overflows(config_.start_address, config_.length) ||
        config_.length < sizeof(FlashLogRecordHeader) ||
        config_.erase_block_size == 0U ||
        config_.write_alignment == 0U ||
        config_.write_alignment > config_.length ||
        config_.max_payload_size == 0U ||
        config_.max_payload_size > (config_.length - sizeof(FlashLogRecordHeader))) {
        status_ = FlashLogStatus::BadConfig;
        return false;
    }

    if ((config_.start_address % config_.erase_block_size) != 0U ||
        (config_.length % config_.erase_block_size) != 0U) {
        status_ = FlashLogStatus::BadConfig;
        return false;
    }

    initialized_ = true;
    append_ready_ = scan();
    if (!append_ready_) {
        return false;
    }

    if (allocate_run_id) {
        if (highest_run_id_ == UINT32_MAX) {
            append_ready_ = false;
            status_ = FlashLogStatus::RunIdExhausted;
            return false;
        }
        run_id_ = highest_run_id_ + 1U;
    }

    return append_ready_;
}

bool FlashLogger::erase_all() {
    if (!initialized_) {
        status_ = FlashLogStatus::NotInitialized;
        return false;
    }

    if (!flash_.erase(config_.start_address, config_.length)) {
        status_ = FlashLogStatus::FlashError;
        return false;
    }

    next_address_ = config_.start_address;
    record_count_ = 0U;
    run_record_count_ = 0U;
    highest_run_id_ = 0U;
    append_ready_ = true;
    full_ = false;
    status_ = FlashLogStatus::Ok;
    return true;
}

bool FlashLogger::append(const void* payload, size_t payload_length,
                         uint32_t timestamp_ms, uint32_t* record_address) {
    return append_typed(payload,
                        payload_length,
                        timestamp_ms,
                        FlashLogPayloadType::Unspecified,
                        0U,
                        record_address);
}

bool FlashLogger::append_typed(const void* payload,
                               size_t payload_length,
                               uint32_t timestamp_ms,
                               FlashLogPayloadType payload_type,
                               uint16_t payload_version,
                               uint32_t* record_address) {
    if (!initialized_) {
        status_ = FlashLogStatus::NotInitialized;
        return false;
    }

    if (!append_ready_) {
        if (full_) {
            status_ = FlashLogStatus::Full;
        }
        return false;
    }

    if ((payload == nullptr && payload_length > 0U) ||
        payload_length > config_.max_payload_size ||
        payload_length > UINT32_MAX) {
        status_ = FlashLogStatus::PayloadTooLarge;
        return false;
    }

    FlashLogRecordHeader header{};
    header.magic = FLASH_LOG_MAGIC;
    header.version = FLASH_LOG_VERSION;
    header.header_size = sizeof(FlashLogRecordHeader);
    header.run_id = run_id_;
    header.sequence = run_record_count_;
    header.timestamp_ms = timestamp_ms;
    header.payload_type = static_cast<uint16_t>(payload_type);
    header.payload_version = payload_version;
    header.payload_length = static_cast<uint32_t>(payload_length);
    header.payload_checksum = fnv1a(payload, payload_length);
    header.commit_marker = FLASH_LOG_UNCOMMITTED;
    header.header_checksum = header_checksum(header);

    const uint32_t total_size = record_size(header);
    if (total_size == 0U ||
        next_address_ > end_address() ||
        total_size > (end_address() - next_address_)) {
        full_ = true;
        latch_append_failure(FlashLogStatus::Full);
        return false;
    }

    if (!blank(next_address_, total_size)) {
        append_ready_ = false;
        return false;
    }

    const uint32_t address = next_address_;
    if (!flash_.write(address,
                      reinterpret_cast<const uint8_t*>(&header),
                      sizeof(header))) {
        latch_append_failure(FlashLogStatus::FlashError);
        return false;
    }

    if (payload_length > 0U &&
        !flash_.write(address + sizeof(header),
                      reinterpret_cast<const uint8_t*>(payload),
                      payload_length)) {
        latch_append_failure(FlashLogStatus::FlashError);
        return false;
    }

    if (config_.verify_writes && !verify_record(address, header)) {
        latch_append_failure(FlashLogStatus::VerifyFailed);
        return false;
    }

    const uint32_t commit = FLASH_LOG_COMMITTED;
    if (!flash_.write(address + offsetof(FlashLogRecordHeader, commit_marker),
                      reinterpret_cast<const uint8_t*>(&commit),
                      sizeof(commit))) {
        latch_append_failure(FlashLogStatus::FlashError);
        return false;
    }

    FlashLogRecordHeader committed_header = header;
    committed_header.commit_marker = FLASH_LOG_COMMITTED;
    if (config_.verify_writes && !verify_record(address, committed_header)) {
        latch_append_failure(FlashLogStatus::VerifyFailed);
        return false;
    }

    if (record_address != nullptr) {
        *record_address = address;
    }

    next_address_ += total_size;
    record_count_++;
    run_record_count_++;
    full_ = (next_address_ >= end_address());
    append_ready_ = !full_;
    status_ = FlashLogStatus::Ok;
    return true;
}

FlashLogCursor FlashLogger::cursor() const {
    FlashLogCursor cursor{};
    cursor.address = config_.start_address;
    return cursor;
}

bool FlashLogger::read_next(FlashLogCursor& cursor,
                            FlashLogRecordHeader& header,
                            void* payload,
                            size_t payload_capacity,
                            size_t* payload_length) {
    if (!initialized_) {
        status_ = FlashLogStatus::NotInitialized;
        return false;
    }

    if (cursor.address == 0U) {
        cursor.address = config_.start_address;
    }

    while (cursor.address < next_address_) {
        const uint32_t current_address = cursor.address;
        if (read_record(current_address, header, payload, payload_capacity, payload_length)) {
            cursor.address = current_address + record_size(header);
            return true;
        }

        if (status_ != FlashLogStatus::Incomplete) {
            return false;
        }

        const uint32_t skipped_size = record_size(header);
        if (skipped_size == 0U || skipped_size > (end_address() - current_address)) {
            status_ = FlashLogStatus::Corrupt;
            return false;
        }

        cursor.address = current_address + skipped_size;
    }

    status_ = FlashLogStatus::NotFound;
    return false;
}

bool FlashLogger::read_record(uint32_t address,
                              FlashLogRecordHeader& header,
                              void* payload,
                              size_t payload_capacity,
                              size_t* payload_length) {
    if (!initialized_) {
        status_ = FlashLogStatus::NotInitialized;
        return false;
    }

    if (address < config_.start_address ||
        address > (end_address() - sizeof(FlashLogRecordHeader))) {
        status_ = FlashLogStatus::NotFound;
        return false;
    }

    if (!read_header(address, header)) {
        return false;
    }

    if (erased_header(header)) {
        status_ = FlashLogStatus::NotFound;
        return false;
    }

    if (!valid_header(header)) {
        status_ = FlashLogStatus::Corrupt;
        return false;
    }

    const uint32_t total_size = record_size(header);
    if (address > end_address() || total_size > (end_address() - address)) {
        status_ = FlashLogStatus::Corrupt;
        return false;
    }

    if (!committed_header(header)) {
        status_ = FlashLogStatus::Incomplete;
        return false;
    }

    return read_payload(address, header, payload, payload_capacity, payload_length);
}

FlashLogInfo FlashLogger::info() const {
    FlashLogInfo log_info{};
    log_info.start_address = config_.start_address;
    log_info.end_address = end_address();
    log_info.next_address = next_address_;
    log_info.used_bytes = (next_address_ >= config_.start_address)
                               ? (next_address_ - config_.start_address)
                               : 0U;
    log_info.record_count = record_count_;
    log_info.run_record_count = run_record_count_;
    log_info.run_id = run_id_;
    log_info.highest_run_id = highest_run_id_;
    log_info.full = full_;
    log_info.initialized = initialized_;
    log_info.append_ready = append_ready_;
    return log_info;
}

FlashLogStatus FlashLogger::status() const {
    return status_;
}

bool FlashLogger::scan() {
    uint32_t address = config_.start_address;
    record_count_ = 0U;
    run_record_count_ = 0U;
    highest_run_id_ = 0U;
    full_ = false;

    while (address <= (end_address() - sizeof(FlashLogRecordHeader))) {
        FlashLogRecordHeader header{};
        if (!read_header(address, header)) {
            return false;
        }

        if (erased_header(header)) {
            next_address_ = address;
            status_ = FlashLogStatus::Ok;
            return true;
        }

        if (!valid_header(header)) {
            next_address_ = address;
            status_ = FlashLogStatus::Corrupt;
            return false;
        }

        const uint32_t total_size = record_size(header);
        if (total_size == 0U ||
            address > end_address() ||
            total_size > (end_address() - address)) {
            next_address_ = address;
            status_ = FlashLogStatus::Corrupt;
            return false;
        }

        if (!committed_header(header)) {
            address += total_size;
            status_ = FlashLogStatus::Incomplete;
            continue;
        }

        if (!payload_checksum_matches(address, header)) {
            next_address_ = address;
            status_ = FlashLogStatus::Corrupt;
            return false;
        }

        if (header.run_id > highest_run_id_) {
            highest_run_id_ = header.run_id;
        }

        if (header.run_id == run_id_) {
            run_record_count_++;
        }

        address += total_size;
        record_count_++;
    }

    next_address_ = end_address();
    full_ = true;
    status_ = FlashLogStatus::Full;
    return false;
}

bool FlashLogger::read_header(uint32_t address, FlashLogRecordHeader& header) {
    if (!flash_.read(address,
                     reinterpret_cast<uint8_t*>(&header),
                     sizeof(header))) {
        status_ = FlashLogStatus::FlashError;
        return false;
    }

    return true;
}

bool FlashLogger::read_payload(uint32_t address,
                               const FlashLogRecordHeader& header,
                               void* payload,
                               size_t payload_capacity,
                               size_t* payload_length) {
    if (payload_length != nullptr) {
        *payload_length = header.payload_length;
    }

    if ((header.payload_length > 0U && payload == nullptr) ||
        payload_capacity < header.payload_length) {
        status_ = FlashLogStatus::PayloadTooLarge;
        return false;
    }

    if (header.payload_length == 0U) {
        if (header.payload_checksum != fnv1a(nullptr, 0U)) {
            status_ = FlashLogStatus::Corrupt;
            return false;
        }

        status_ = FlashLogStatus::Ok;
        return true;
    }

    if (!flash_.read(address + sizeof(FlashLogRecordHeader),
                     reinterpret_cast<uint8_t*>(payload),
                     header.payload_length)) {
        status_ = FlashLogStatus::FlashError;
        return false;
    }

    if (fnv1a(payload, header.payload_length) != header.payload_checksum) {
        status_ = FlashLogStatus::Corrupt;
        return false;
    }

    status_ = FlashLogStatus::Ok;
    return true;
}

bool FlashLogger::payload_checksum_matches(uint32_t address,
                                           const FlashLogRecordHeader& header) {
    uint8_t buffer[VERIFY_CHUNK];
    uint32_t remaining = header.payload_length;
    uint32_t offset = 0U;
    uint32_t checksum = FNV1A_OFFSET;

    while (remaining > 0U) {
        uint32_t chunk = VERIFY_CHUNK;
        if (chunk > remaining) {
            chunk = remaining;
        }

        if (!flash_.read(address + sizeof(FlashLogRecordHeader) + offset,
                         buffer,
                         chunk)) {
            status_ = FlashLogStatus::FlashError;
            return false;
        }

        checksum = fnv1a_update(checksum, buffer, chunk);
        offset += chunk;
        remaining -= chunk;
    }

    return checksum == header.payload_checksum;
}

bool FlashLogger::verify_record(uint32_t address,
                                const FlashLogRecordHeader& expected_header) {
    FlashLogRecordHeader readback{};
    if (!read_header(address, readback)) {
        return false;
    }

    if (std::memcmp(&readback, &expected_header, sizeof(readback)) != 0) {
        return false;
    }

    return payload_checksum_matches(address, expected_header);
}

bool FlashLogger::blank(uint32_t address, uint32_t length) {
    uint8_t buffer[BLANK_CHECK_CHUNK];
    uint32_t remaining = length;
    uint32_t offset = 0U;

    while (remaining > 0U) {
        uint32_t chunk = BLANK_CHECK_CHUNK;
        if (chunk > remaining) {
            chunk = remaining;
        }

        if (!flash_.read(address + offset, buffer, chunk)) {
            status_ = FlashLogStatus::FlashError;
            return false;
        }

        for (uint32_t i = 0U; i < chunk; i++) {
            if (buffer[i] != FLASH_LOG_ERASED_BYTE) {
                status_ = FlashLogStatus::Corrupt;
                return false;
            }
        }

        offset += chunk;
        remaining -= chunk;
    }

    return true;
}

bool FlashLogger::erased_header(const FlashLogRecordHeader& header) const {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&header);
    for (size_t i = 0U; i < sizeof(header); i++) {
        if (bytes[i] != FLASH_LOG_ERASED_BYTE) {
            return false;
        }
    }

    return true;
}

bool FlashLogger::valid_header(const FlashLogRecordHeader& header) const {
    return header.magic == FLASH_LOG_MAGIC &&
           header.version == FLASH_LOG_VERSION &&
           header.header_size == sizeof(FlashLogRecordHeader) &&
           header.payload_length <= config_.max_payload_size &&
           (header.commit_marker == FLASH_LOG_UNCOMMITTED ||
            header.commit_marker == FLASH_LOG_COMMITTED) &&
           header_checksum(header) == header.header_checksum;
}

bool FlashLogger::committed_header(const FlashLogRecordHeader& header) const {
    return header.commit_marker == FLASH_LOG_COMMITTED;
}

uint32_t FlashLogger::record_size(const FlashLogRecordHeader& header) const {
    if (header.header_size > UINT32_MAX - header.payload_length) {
        return 0U;
    }

    return aligned_size(header.header_size + header.payload_length);
}

uint32_t FlashLogger::aligned_size(uint32_t size) const {
    const uint32_t alignment = config_.write_alignment;
    const uint32_t remainder = size % alignment;
    if (remainder == 0U) {
        return size;
    }

    const uint32_t padding = alignment - remainder;
    if (size > (UINT32_MAX - padding)) {
        return 0U;
    }

    return size + padding;
}

uint32_t FlashLogger::end_address() const {
    return config_.start_address + config_.length;
}

void FlashLogger::latch_append_failure(FlashLogStatus status) {
    append_ready_ = false;
    status_ = status;
}

uint32_t FlashLogger::fnv1a(const void* data, size_t length) {
    return fnv1a_update(FNV1A_OFFSET, data, length);
}

uint32_t FlashLogger::fnv1a_update(uint32_t hash, const void* data, size_t length) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data);
    for (size_t i = 0U; i < length; i++) {
        hash ^= bytes[i];
        hash *= FNV1A_PRIME;
    }

    return hash;
}

uint32_t FlashLogger::header_checksum(const FlashLogRecordHeader& header) {
    FlashLogRecordHeader copy = header;
    copy.header_checksum = 0U;
    copy.commit_marker = FLASH_LOG_UNCOMMITTED;
    return fnv1a(&copy, sizeof(copy));
}