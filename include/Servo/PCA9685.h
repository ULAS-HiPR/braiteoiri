#ifndef PCA9685_H
#define PCA9685_H

#include <stdint.h>
#include "Servo.h"
#include <I2C/I2C_Handler.h>

class PCA9685Servo : public Servo {
    public:
        explicit PCA9685Servo(I2C_Handler& i2c_handler, uint8_t channel)
            : i2c_handler(i2c_handler), channel(channel) {}

        bool init() override;
        bool set_position(int8_t position) override;

    private:
        void reset();
        void set_pwm_freq(uint16_t freq);
        void set_pwm(uint8_t channel, uint16_t on, uint16_t off);
        uint16_t angle_to_pwm(int8_t angle);

        I2C_Handler& i2c_handler;

        uint8_t channel;

        static constexpr uint8_t PCA9685_ADDRESS      = 0x40;
        static constexpr uint8_t MODE1                = 0x00;
        static constexpr uint8_t PRESCALE             = 0xFE;
        static constexpr uint8_t LED0_ON_L            = 0x06;
};

#endif