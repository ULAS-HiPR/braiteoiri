#include <Baro/BMP390.h>

extern "C" BMP3_INTF_RET_TYPE bmp3_spi_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    //SPI* spi = reinterpret_cast<SPI*>(intf_ptr);
    //spi->read_no_cs( reg_addr, data, len);
    //tbd
    return 0;
}

extern "C" BMP3_INTF_RET_TYPE bmp3_spi_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    //SPI* spi = reinterpret_cast<SPI*>(intf_ptr);
    //spi->write_no_cs(reg_addr, data, len); 
    //tbd
    return 0;
}

extern "C" void delay_us(uint32_t period, void *intf_ptr) {
    printf("Delay: %ld\n", period);
    printf("fake sleep cause debugger not like");
}

BMP390::BMP390(I2C_Handler& i2c_handler) {
    printf("Barometer created\n");

    baro.intf = BMP3_SPI_INTF;
    //baro.read = bmp3_spi_read;
    //baro.write = bmp3_spi_write;
    //baro.delay_us = delay_us;
    //baro.intf_ptr = spi;
    baro.dummy_byte = 0;

    printf("settings init\n");

    // Initialize the sensor
    //spi->cs_select(cs);
    ////int8_t rslt = bmp3_soft_reset(&baro);
    ////spi->cs_deselect(cs);
    //bmp3_check_rslt("bmp3_soft_reset", rslt);
    
    //spi->cs_select(cs);
    //int8_t rslt = bmp3_init(&baro);
    //uint8_t tx = 0x00;
    //uint8_t rx[2];
    //
    //spi->read_no_cs(tx, rx, 2);
    //spi->cs_deselect(cs);

    //for (int i = 0; i < 2; i++) {
    //    printf("rx[%d]: %02x\n", i, rx[i]);
    //}
    //bmp3_check_rslt("bmp3_init", rslt);
    
    printf("Barometer initialized\n");
  
}

BMP390::~BMP390(){
    printf("Barometer destroyed\n");
}

void BMP390::read(){
    int8_t rslt;

    uint16_t settings_sel = 0;
    
    uint8_t sensor_comp = 0;

   
    settings.temp_en = BMP3_ENABLE;
    settings_sel |= BMP3_SEL_TEMP_EN;
    sensor_comp |= BMP3_TEMP;

    settings.press_en = BMP3_ENABLE;
    settings_sel |= BMP3_SEL_PRESS_EN;
    sensor_comp |= BMP3_PRESS;

    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &baro);
    bmp3_check_rslt("bmp3_set_sensor_settings", rslt);
    struct bmp3_data data;
    //spi->cs_select(cs);
    rslt = bmp3_get_sensor_data(sensor_comp, &data, &baro);
    //spi->cs_deselect(cs);
    bmp3_check_rslt("bmp3_get_sensor_data", rslt);
    printf("Pressure: %f\n", data.pressure);
    printf("Temperature: %f\n", data.temperature);

}

bool BMP390::update(baro_data* data){
    read();
    // Process altitude data
    
    printf("Barometer update\n");
    data->altitude = 10;
    return true;
}

void BMP390::bmp3_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMP3_OK:

            
            break;
        case BMP3_E_NULL_PTR:
            printf("API [%s] Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BMP3_E_COMM_FAIL:
            printf("API [%s] Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BMP3_E_INVALID_LEN:
            printf("API [%s] Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BMP3_E_DEV_NOT_FOUND:
            printf("API [%s] Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BMP3_E_CONFIGURATION_ERR:
            printf("API [%s] Error [%d] : Configuration Error\r\n", api_name, rslt);
            break;
        case BMP3_W_SENSOR_NOT_ENABLED:
            printf("API [%s] Error [%d] : Warning when Sensor not enabled\r\n", api_name, rslt);
            break;
        case BMP3_W_INVALID_FIFO_REQ_FRAME_CNT:
            printf("API [%s] Error [%d] : Warning when Fifo watermark level is not in limit\r\n", api_name, rslt);
            break;
        default:
            printf("API [%s] Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}
