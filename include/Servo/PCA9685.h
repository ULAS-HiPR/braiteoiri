#ifndef PCA9685_H
#define PCA9685_H

#include <stdint.h>
#include "servo.h"
#include <I2C/I2C_Handler.h>

class PCA9685Servo : public Servo {
    public:
        static constexpr uint8_t DEFAULT_ADDRESS      = 0x40;

        explicit PCA9685Servo(I2C_Handler& i2c_handler, uint8_t channel, uint8_t address = DEFAULT_ADDRESS)
            : i2c_handler(i2c_handler), channel(channel), address(address) {}

        bool init() override;
        bool set_position(int16_t position) override;

    private:
        void reset();
        void set_pwm_freq(uint16_t freq);
        void set_pwm(uint8_t channel, uint16_t on, uint16_t off);
        uint16_t angle_to_pwm(int16_t angle);

        I2C_Handler& i2c_handler;

        uint8_t channel;
        uint8_t address;

        static constexpr uint8_t MODE1                = 0x00;
        static constexpr uint8_t MODE2                = 0x01;
        static constexpr uint8_t PRESCALE             = 0xFE;
        static constexpr uint8_t LED0_ON_L            = 0x06;
        static constexpr uint8_t MODE1_RESTART        = 0x80;
        static constexpr uint8_t MODE1_AI             = 0x20;
        static constexpr uint8_t MODE1_SLEEP          = 0x10;
        static constexpr uint8_t MODE2_OUTDRV         = 0x04;
};

#endif
