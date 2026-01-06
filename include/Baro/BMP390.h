#include <Baro/baro.h>
#include <I2C/I2C_Handler.h>
#include <cstdio>
#include "../src/Baro/BMP390/bmp3_defs.h"
#include "../src/Baro/BMP390/bmp3.h"

class BMP390 : public Baro {
public:
    BMP390(I2C_Handler& i2c_handler);
    bool init() override;
    ~BMP390();
    bool update(baro_data* data) override;
private:
    int cs;
    void read();
    void process();
    struct bmp3_dev baro;
    struct bmp3_settings settings;
    void bmp3_check_rslt(const char api_name[], int8_t rslt);
};



