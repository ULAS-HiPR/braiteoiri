#ifndef FLASH_LOGGER_H
#define FLASH_LOGGER_H

#include <cstddef>
#include <cstdint>
#include "flash.h"

#define FLASH_LOG_MAGIC       0x48495052U
#define FLASH_LOG_VERSION     2U
#define FLASH_LOG_ERASED_BYTE 0xFFU
#define FLASH_LOG_UNCOMMITTED 0xFFFFFFFFU
#define FLASH_LOG_COMMITTED   0x00000000U
#define FLASH_LOG_PAYLOAD_VERSION 1U

enum class FlashLogPayloadType : uint16_t {
    Unspecified = 0,
    FlightData = 1,
    SecondaryFlightData = 2
};

struct FlashLogConfig {
    uint32_t start_address;
    uint32_t length;
    uint32_t erase_block_size;
    uint32_t write_alignment;
    uint32_t max_payload_size;
    bool verify_writes;
};

struct __attribute__((packed)) FlashLogRecordHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t header_size;
    uint32_t run_id;
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint16_t payload_type;
    uint16_t payload_version;
    uint32_t payload_length;
    uint32_t payload_checksum;
    uint32_t header_checksum;
    uint32_t commit_marker;
};

struct FlashLogInfo {
    uint32_t start_address;
    uint32_t end_address;
    uint32_t next_address;
    uint32_t used_bytes;
    uint32_t record_count;
    uint32_t run_record_count;
    uint32_t run_id;
    uint32_t highest_run_id;
    bool full;
    bool initialized;
    bool append_ready;
};

struct FlashLogCursor {
    uint32_t address;
};

enum class FlashLogStatus : uint8_t {
    Ok = 0,
    NotInitialized,
    BadConfig,
    FlashError,
    Full,
    PayloadTooLarge,
    Corrupt,
    VerifyFailed,
    NotFound,
    RunIdExhausted,
    Incomplete
};

class FlashLogger {
    public:
        explicit FlashLogger(Flash& flash);

        bool begin(const FlashLogConfig& config);
        bool begin(const FlashLogConfig& config, uint32_t run_id);
        bool begin(uint32_t start_address, uint32_t length);
        bool begin(uint32_t start_address, uint32_t length, uint32_t run_id);
        bool erase_all();

        bool append(const void* payload, size_t payload_length,
                    uint32_t timestamp_ms, uint32_t* record_address = nullptr);
        bool append_typed(const void* payload,
                          size_t payload_length,
                          uint32_t timestamp_ms,
                          FlashLogPayloadType payload_type,
                          uint16_t payload_version = FLASH_LOG_PAYLOAD_VERSION,
                          uint32_t* record_address = nullptr);

        template<typename T>
        bool append_record(const T& record, uint32_t timestamp_ms,
                           uint32_t* record_address = nullptr) {
            return append(&record, sizeof(T), timestamp_ms, record_address);
        }

        template<typename T>
        bool append_record_typed(const T& record,
                                 uint32_t timestamp_ms,
                                 FlashLogPayloadType payload_type,
                                 uint16_t payload_version = FLASH_LOG_PAYLOAD_VERSION,
                                 uint32_t* record_address = nullptr) {
            return append_typed(&record, sizeof(T), timestamp_ms,
                                payload_type, payload_version, record_address);
        }

        FlashLogCursor cursor() const;
        bool read_next(FlashLogCursor& cursor,
                       FlashLogRecordHeader& header,
                       void* payload,
                       size_t payload_capacity,
                       size_t* payload_length = nullptr);

        bool read_record(uint32_t address,
                         FlashLogRecordHeader& header,
                         void* payload,
                         size_t payload_capacity,
                         size_t* payload_length = nullptr);

        FlashLogInfo info() const;
        FlashLogStatus status() const;

        static FlashLogConfig default_mx25_config(uint32_t start_address,
                                                  uint32_t length);

    private:
        Flash& flash_;
        FlashLogConfig config_{};
        uint32_t next_address_{0U};
        uint32_t record_count_{0U};
        uint32_t run_record_count_{0U};
        uint32_t run_id_{0U};
        uint32_t highest_run_id_{0U};
        FlashLogStatus status_{FlashLogStatus::NotInitialized};
        bool initialized_{false};
        bool append_ready_{false};
        bool full_{false};

        bool begin_with_mode(const FlashLogConfig& config,
                             uint32_t run_id,
                             bool allocate_run_id);
        bool scan();
        bool read_header(uint32_t address, FlashLogRecordHeader& header);
        bool read_payload(uint32_t address,
                          const FlashLogRecordHeader& header,
                          void* payload,
                          size_t payload_capacity,
                          size_t* payload_length);
        bool payload_checksum_matches(uint32_t address,
                                      const FlashLogRecordHeader& header);
        bool verify_record(uint32_t address,
                           const FlashLogRecordHeader& expected_header);
        bool blank(uint32_t address, uint32_t length);
        bool erased_header(const FlashLogRecordHeader& header) const;
        bool valid_header(const FlashLogRecordHeader& header) const;
        bool committed_header(const FlashLogRecordHeader& header) const;
        uint32_t record_size(const FlashLogRecordHeader& header) const;
        uint32_t aligned_size(uint32_t size) const;
        uint32_t end_address() const;
        void latch_append_failure(FlashLogStatus status);

        static uint32_t fnv1a(const void* data, size_t length);
        static uint32_t fnv1a_update(uint32_t hash, const void* data, size_t length);
        static uint32_t header_checksum(const FlashLogRecordHeader& header);
};

#endif
