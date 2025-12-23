#include "IMU.h"
#include <vector>

class MockIMU : public IMU {
public:
    explicit MockIMU(std::vector<imu_data> data)
        : data(data) {}

    bool init() override { index = 0; return true; }

    bool read(imu_data& out) override {
        if (index >= data.size()) return false;
        out = data[index++];
        return true;
    }

private:
    std::vector<imu_data> data;
    size_t index = 0;
};