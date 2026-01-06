#include "../sensor.h"
#include <stdint.h>

//10 bytes update
struct baro_data
{
    int32_t pressure{101325};
    float temperature{0};
    float altitude{0};
};

class Baro : public Sensor<baro_data> {
public:
    virtual bool init() = 0;
    bool update(baro_data* data) override;
};
