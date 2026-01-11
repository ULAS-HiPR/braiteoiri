#include <Baro/baro.h>
#include <I2C/I2C_Handler.h>
#include <SPI/SPI_Handler.h>
#include <cstdio>
#include "../src/Baro/BMP390/bmp3_defs.h"
#include "../src/Baro/BMP390/bmp3.h"

class BMP390 : public Baro {
public:
    BMP390(I2C_Handler *i2c_handler);
    BMP390(SPI_Handler *spi_handler);
    bool init() override;
    ~BMP390();
    bool update(baro_data* data) override;
private:
    int interface_type{-1}; //-1 = not set, 0 = I2C, 1 = SPI
    I2C_Handler* i2c{nullptr};
    SPI_Handler* spi{nullptr};
    int cs;
    void read();
    void process();
    struct bmp3_dev baro;
    struct bmp3_settings settings;
    void bmp3_check_rslt(const char api_name[], int8_t rslt);
};



