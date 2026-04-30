#include "../../../include/Servo/PCA9685.h"
#include <servo_debug.h>
#include <cmath>

bool PCA9685Servo::init() {
    servo_debug.stage = SERVO_DEBUG_STAGE_PCA_INIT_START;
    servo_debug.pca9685_address = address;
    servo_debug.servo_channel = channel;
    servo_debug.pca9685_init_count++;

    servo_debug.stage = SERVO_DEBUG_STAGE_PCA_RESET;
    reset();

    servo_debug.stage = SERVO_DEBUG_STAGE_PCA_MODE2;
    uint8_t mode2[2] = {MODE2, MODE2_OUTDRV};
    i2c_handler.write(address, mode2, 2);

    set_pwm_freq(50);
    servo_debug.stage = SERVO_DEBUG_STAGE_PCA_INIT_DONE;
    return true;
}

bool PCA9685Servo::set_position(int16_t position) {
    if (position < 0) position = 0;
    if (position > 180) position = 180;

    uint16_t pwm = angle_to_pwm(position);
    servo_debug.stage = SERVO_DEBUG_STAGE_SERVO_SET;
    servo_debug.servo_set_count++;
    servo_debug.servo_channel = channel;
    servo_debug.servo_angle = position;
    servo_debug.servo_pwm = pwm;

    set_pwm(channel, 0, pwm);
    current_position = position;
    return true;
}

void PCA9685Servo::reset() {
    uint8_t data[2] = {MODE1, 0x00};
    i2c_handler.write(address, data, 2);
}

void PCA9685Servo::set_pwm_freq(uint16_t freq) {
    servo_debug.stage = SERVO_DEBUG_STAGE_PCA_FREQ_START;

    float prescaleval = 25000000.0;
    prescaleval /= 4096.0;
    prescaleval /= freq;
    prescaleval -= 1.0;

    uint8_t prescale = static_cast<uint8_t>(std::floor(prescaleval + 0.5));

    uint8_t oldmode = 0;
    i2c_handler.read(address, MODE1, &oldmode, 1);
    servo_debug.pca9685_mode1_before_prescale = oldmode;

    uint8_t sleep[2] = {MODE1, static_cast<uint8_t>((oldmode & ~MODE1_RESTART) | MODE1_SLEEP)};
    i2c_handler.write(address, sleep, 2);

    uint8_t data[2] = {PRESCALE, prescale};
    servo_debug.pca9685_prescale = prescale;
    i2c_handler.write(address, data, 2);

    uint8_t wake[2] = {MODE1, static_cast<uint8_t>((oldmode & ~MODE1_SLEEP) | MODE1_AI)};
    i2c_handler.write(address, wake, 2);
    i2c_handler.delay_ms(5);

    uint8_t restart[2] = {MODE1, static_cast<uint8_t>(wake[1] | MODE1_RESTART)};
    i2c_handler.write(address, restart, 2);
    servo_debug.stage = SERVO_DEBUG_STAGE_PCA_FREQ_DONE;
}

void PCA9685Servo::set_pwm(uint8_t ch, uint16_t on, uint16_t off) {
    servo_debug.stage = SERVO_DEBUG_STAGE_PWM_WRITE;
    servo_debug.servo_channel = ch;
    servo_debug.pwm_on = on;
    servo_debug.pwm_off = off;

    uint8_t reg = LED0_ON_L + 4 * ch;
    uint8_t data[5] = {
        reg,
        static_cast<uint8_t>(on & 0xFF),
        static_cast<uint8_t>(on >> 8),
        static_cast<uint8_t>(off & 0xFF),
        static_cast<uint8_t>(off >> 8)
    };
    i2c_handler.write(address, data, 5);
}

uint16_t PCA9685Servo::angle_to_pwm(int16_t angle) {
    const uint16_t min_pwm = 102;
    const uint16_t max_pwm = 512;
    return min_pwm + ((max_pwm - min_pwm) * angle / 180);
}
