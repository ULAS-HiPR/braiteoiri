#include <IMU/LSM6DSO32.h>
#include <SPI/SPI_Handler.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <map>
#include <utility>
#include <vector>

namespace {

constexpr std::uint8_t kWhoAmIReg = 0x0F;
constexpr std::uint8_t kCtrl1XlReg = 0x10;
constexpr std::uint8_t kCtrl2GReg = 0x11;
constexpr std::uint8_t kCtrl3CReg = 0x12;
constexpr std::uint8_t kOutTempLReg = 0x20;

struct WriteRecord {
    std::uint8_t reg;
    std::vector<std::uint8_t> data;
};

class MockSPI final : public SPI_Handler {
public:
    void read(int, std::uint8_t reg, std::uint8_t* buf, std::uint16_t len) override
    {
        const auto it = reads.find(reg);
        if (it == reads.end()) {
            std::fill(buf, buf + len, 0U);
            return;
        }

        for (std::uint16_t index = 0; index < len; ++index) {
            buf[index] = (index < it->second.size()) ? it->second[index] : 0U;
        }
    }

    void read_no_cs(std::uint8_t, std::uint8_t*, std::uint16_t) override {}

    void write(int, std::uint8_t reg, std::uint8_t* buf, std::uint16_t len) override
    {
        writes.push_back({reg, std::vector<std::uint8_t>(buf, buf + len)});
    }

    void write_no_cs(std::uint8_t, const std::uint8_t*, std::uint16_t) override {}

    void cs_select(int) override {}
    void cs_deselect(int) override {}
    void delay_ms(int ms) override { delays.push_back(ms); }

    std::map<std::uint8_t, std::vector<std::uint8_t>> reads;
    std::vector<WriteRecord> writes;
    std::vector<int> delays;
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
    spi.reads[kCtrl3CReg] = {0x00};
    spi.reads[kWhoAmIReg] = {0x6C};
    spi.reads[kOutTempLReg] = {
        0x00, 0x01,
        0xE8, 0x03,
        0x18, 0xFC,
        0x00, 0x00,
        0x00, 0x04,
        0x00, 0x08,
        0x00, 0xFC,
    };

    LSM6DSO32 imu(spi);
    require(imu.init(), "LSM6DSO32 init should succeed");

    require(spi.writes.size() == 4, "LSM6DSO32 init should program four registers");
    require(
        (spi.writes[0].reg == kCtrl3CReg) && (spi.writes[0].data[0] == 0x01),
        "LSM6DSO32 should issue a software reset");
    require(
        (spi.writes[1].reg == kCtrl3CReg) && (spi.writes[1].data[0] == 0x44),
        "LSM6DSO32 should enable BDU and address auto-increment");
    require(
        (spi.writes[2].reg == kCtrl1XlReg) && (spi.writes[2].data[0] == 0x64),
        "LSM6DSO32 should configure the accelerometer for 416 Hz and +/-32 g");
    require(
        (spi.writes[3].reg == kCtrl2GReg) && (spi.writes[3].data[0] == 0x6C),
        "LSM6DSO32 should configure the gyroscope for 416 Hz and 2000 dps");

    imu_data sample{};
    require(imu.update(&sample), "LSM6DSO32 update should succeed after init");

    require(nearly_equal(sample.acceleration.x, 0.999f, 0.01f), "Unexpected X acceleration");
    require(nearly_equal(sample.acceleration.y, 1.998f, 0.02f), "Unexpected Y acceleration");
    require(nearly_equal(sample.acceleration.z, -0.999f, 0.01f), "Unexpected Z acceleration");
    require(sample.gyro.x == 70, "Unexpected X gyro");
    require(sample.gyro.y == -70, "Unexpected Y gyro");
    require(sample.gyro.z == 0, "Unexpected Z gyro");
    require(sample.temperature == 26, "Unexpected temperature conversion");

    return 0;
}
