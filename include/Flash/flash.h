#include <stdint.h>

class Flash {
    public:
        //make virtal when driver is done
        Flash();
        ~Flash() = default;
        bool init();
        bool write(uint32_t address, const uint8_t* data, size_t length);
        bool read(uint32_t address, uint8_t* buffer, size_t length);
        bool erase(uint32_t address, size_t length);

    private:
        uint32_t last_written_address;
};