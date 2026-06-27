#ifndef MX25L128_H
#define MX25L128_H

#include <cstdint>
#include <cstddef>
#include <SPI/SPI_Handler.h>
#include "flash.h"

#define MX25_FLASH_SIZE     0x1000000  // 16MB
#define MX25_SECTOR_SIZE    0x1000     // 4KB is the minimum that can be erased
#define MX25_PAGE_SIZE      0x100      // 256B is the minimum that can be written

#define MX25_JEDEC_ID       0xC22018
#define W25Q128_JEDEC_ID    0xEF4018

// from MX25_CMD.h (command bytes))
#define MX25_CMD_RDID       0x9F // read chip ID
#define MX25_CMD_RDSR       0x05 // read status register
#define MX25_CMD_WREN       0x06 // enable write
#define MX25_CMD_SE         0x20 // sector erase
#define MX25_CMD_PP         0x02 // page program/writing
#define MX25_CMD_READ       0x03 // read data
#define MX25_CMD_DP         0xB9 // deep power down
#define MX25_CMD_RDP        0xAB // release from deep power down

#define MX25_WIP_MASK       0x01  // write in progress (important to know wether chip is busy)

class MX25L128 : public Flash{
    public:
        explicit MX25L128(SPI_Handler& spi) : _spi(spi) {}

        bool init() override;                                     // verify chip ID, wake up
        uint32_t jedec_id();
        bool write(uint32_t address, const uint8_t* data, size_t length) override;
        bool read(uint32_t address, uint8_t* buffer, size_t length) override;
        bool erase(uint32_t address, size_t length) override;     // erases minimum 4KB sectors

    private:
        SPI_Handler& _spi;

        bool     send_cmd(uint8_t cmd);
        bool     send_addr(uint32_t address);
        bool     wait_ready(uint32_t timeout_ms);  // poll WIP bit
        bool     write_enable();
        uint8_t  read_status();
        bool     page_program(uint32_t address, const uint8_t* data, size_t length);
        bool     sector_erase(uint32_t address);
};

#endif
