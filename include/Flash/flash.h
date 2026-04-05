#include <cstddef>
#include <stdint.h>

class Flash {
    public:
        virtual bool init();
        virtual bool write(uint32_t address, const uint8_t* data, size_t length);
        virtual bool read(uint32_t address, uint8_t* buffer, size_t length);
        virtual bool erase(uint32_t address, size_t length);

    private:
        uint32_t last_written_address;
};