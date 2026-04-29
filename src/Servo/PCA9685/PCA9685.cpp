#include "../../../include/Servo/PCA9685.h"
#include <cmath>

bool PCA9685Servo::init() {
    reset();
    set_pwm_freq(50);
    return true;
}

bool PCA9685Servo::set_position(int8_t position) {
    if (position < 0) position = 0;
    if (position > 180) position = 180;

    uint16_t pwm = angle_to_pwm(position);
    set_pwm(channel, 0, pwm);
    current_position = position;
    return true;
}

void PCA9685Servo::reset() {
    uint8_t data[2] = {MODE1, 0x00};
    i2c_handler.write(PCA9685_ADDRESS, data, 2);
}

void PCA9685Servo::set_pwm_freq(uint16_t freq) {
    float prescaleval = 25000000.0;
    prescaleval /= 4096.0;
    prescaleval /= freq;
    prescaleval -= 1.0;

    uint8_t prescale = static_cast<uint8_t>(std::floor(prescaleval + 0.5));

    uint8_t data[2] = {PRESCALE, prescale};
    i2c_handler.write(PCA9685_ADDRESS, data, 2);
}

void PCA9685Servo::set_pwm(uint8_t ch, uint16_t on, uint16_t off) {
    uint8_t reg = LED0_ON_L + 4 * ch;
    uint8_t data[5] = {
        reg,
        static_cast<uint8_t>(on & 0xFF),
        static_cast<uint8_t>(on >> 8),
        static_cast<uint8_t>(off & 0xFF),
        static_cast<uint8_t>(off >> 8)
    };
    i2c_handler.write(PCA9685_ADDRESS, data, 5);
}

uint16_t PCA9685Servo::angle_to_pwm(int8_t angle) {
    const uint16_t min_pwm = 102;
    const uint16_t max_pwm = 512;
    return min_pwm + ((max_pwm - min_pwm) * angle / 180);
}