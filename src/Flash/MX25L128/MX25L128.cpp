#include <Flash/MX25L128.h>

bool MX25L128::init() {
    cs_high();

    // wake from deep power down just in case
    send_cmd(MX25_CMD_RDP);
    _spi.delay_ms(1);

    // read chip ID and verify it matches expected 0xC22018
    uint8_t id[3];
    cs_low();
    send_cmd(MX25_CMD_RDID);
    _spi.read(0, 0, id, 3);
    cs_high();

    uint32_t chip_id = ((uint32_t)id[0] << 16) |
                       ((uint32_t)id[1] <<  8) |
                        (uint32_t)id[2];

    return chip_id == 0xC22018;  // false if wrong chip or no chip
}

bool MX25L128::read(uint32_t address, uint8_t* buffer, size_t length) {
    if (address + length > MX25_FLASH_SIZE) return false;

    cs_low();
    send_cmd(MX25_CMD_READ);
    send_addr(address);
    _spi.read(0, 0, buffer, length);
    cs_high();

    return true;
}

bool MX25L128::erase(uint32_t address, size_t length) {
    // erase all sectors that overlap the requested range
    uint32_t sector_start = address & ~(MX25_SECTOR_SIZE - 1);
    uint32_t sector_end   = (address + length + MX25_SECTOR_SIZE - 1)
                            & ~(MX25_SECTOR_SIZE - 1);

    for (uint32_t addr = sector_start; addr < sector_end; addr += MX25_SECTOR_SIZE) {
        if (!sector_erase(addr)) return false;
    }
    return true;
}

bool MX25L128::write(uint32_t address, const uint8_t* data, size_t length) {
    // write must be broken into 256 byte pages
    while (length > 0) {
        // calculate how many bytes fit in this page
        uint32_t page_offset = address % MX25_PAGE_SIZE;
        size_t   chunk = MX25_PAGE_SIZE - page_offset;
        if (chunk > length) chunk = length;

        if (!page_program(address, data, chunk)) return false;

        address += chunk;
        data    += chunk;
        length  -= chunk;
    }
    return true;
}

// ---- Private helpers ----

bool MX25L128::write_enable() {
    cs_low();
    send_cmd(MX25_CMD_WREN);
    cs_high();
    return (read_status() & 0x02) != 0;  // WEL bit should be set
}

bool MX25L128::wait_ready(uint32_t timeout_ms) {
    while (timeout_ms--) {
        if (!(read_status() & MX25_WIP_MASK)) return true;
        _spi.delay_ms(1);
    }
    return false;  // timed out — chip stuck
}

uint8_t MX25L128::read_status() {
    uint8_t status;
    cs_low();
    send_cmd(MX25_CMD_RDSR);
    _spi.read(0, 0, &status, 1);
    cs_high();
    return status;
}

bool MX25L128::sector_erase(uint32_t address) {
    if (!write_enable()) return false;
    cs_low();
    send_cmd(MX25_CMD_SE);
    send_addr(address);
    cs_high();
    return wait_ready(500);  // sector erase takes up to 400ms per MX25_DEF.h
}

bool MX25L128::page_program(uint32_t address, const uint8_t* data, size_t length) {
    if (!write_enable()) return false;
    cs_low();
    send_cmd(MX25_CMD_PP);
    send_addr(address);
    _spi.write(0, const_cast<uint8_t*>(data), length);
    cs_high();
    return wait_ready(10);  // page program takes up to 1.5ms
}

void MX25L128::send_cmd(uint8_t cmd) {
    _spi.write(0, &cmd, 1);
}

void MX25L128::send_addr(uint32_t address) {
    uint8_t addr[3] = {
        (uint8_t)(address >> 16),
        (uint8_t)(address >>  8),
        (uint8_t)(address >>  0)
    };
    _spi.write(0, addr, 3);
}