#include "../../../include/Servo/PCA9685.h"

#ifdef BRAITEOIRI_USE_SERVO_DEBUG
#include <servo_debug.h>
#endif

#include <cmath>

namespace {

void debug_init_start(uint8_t address, uint8_t channel) {
#ifdef BRAITEOIRI_USE_SERVO_DEBUG
    servo_debug.stage = SERVO_DEBUG_STAGE_PCA_INIT_START;
    servo_debug.pca9685_address = address;
    servo_debug.servo_channel = channel;
    servo_debug.pca9685_init_count++;
#else
    (void)address;
    (void)channel;
#endif
}

void debug_pca_reset() {
#ifdef BRAITEOIRI_USE_SERVO_DEBUG
    servo_debug.stage = SERVO_DEBUG_STAGE_PCA_RESET;
#endif
}

void debug_pca_mode2() {
#ifdef BRAITEOIRI_USE_SERVO_DEBUG
    servo_debug.stage = SERVO_DEBUG_STAGE_PCA_MODE2;
#endif
}

void debug_pca_init_done() {
#ifdef BRAITEOIRI_USE_SERVO_DEBUG
    servo_debug.stage = SERVO_DEBUG_STAGE_PCA_INIT_DONE;
#endif
}

void debug_servo_set(uint8_t channel, int16_t position, uint16_t pwm) {
#ifdef BRAITEOIRI_USE_SERVO_DEBUG
    servo_debug.stage = SERVO_DEBUG_STAGE_SERVO_SET;
    servo_debug.servo_set_count++;
    servo_debug.servo_channel = channel;
    servo_debug.servo_angle = position;
    servo_debug.servo_pwm = pwm;
#else
    (void)channel;
    (void)position;
    (void)pwm;
#endif
}

void debug_freq_start() {
#ifdef BRAITEOIRI_USE_SERVO_DEBUG
    servo_debug.stage = SERVO_DEBUG_STAGE_PCA_FREQ_START;
#endif
}

void debug_mode1_before_prescale(uint8_t oldmode) {
#ifdef BRAITEOIRI_USE_SERVO_DEBUG
    servo_debug.pca9685_mode1_before_prescale = oldmode;
#else
    (void)oldmode;
#endif
}

void debug_prescale(uint8_t prescale) {
#ifdef BRAITEOIRI_USE_SERVO_DEBUG
    servo_debug.pca9685_prescale = prescale;
#else
    (void)prescale;
#endif
}

void debug_freq_done() {
#ifdef BRAITEOIRI_USE_SERVO_DEBUG
    servo_debug.stage = SERVO_DEBUG_STAGE_PCA_FREQ_DONE;
#endif
}

void debug_pwm_write(uint8_t channel, uint16_t on, uint16_t off) {
#ifdef BRAITEOIRI_USE_SERVO_DEBUG
    servo_debug.stage = SERVO_DEBUG_STAGE_PWM_WRITE;
    servo_debug.servo_channel = channel;
    servo_debug.pwm_on = on;
    servo_debug.pwm_off = off;
#else
    (void)channel;
    (void)on;
    (void)off;
#endif
}

} // namespace

bool PCA9685Servo::init() {
    debug_init_start(address, channel);

    debug_pca_reset();
    if (!reset()) return false;

    debug_pca_mode2();
    uint8_t mode2[2] = {MODE2, MODE2_OUTDRV};
    if (!i2c_handler.write(address, mode2, 2)) return false;

    if (!set_pwm_freq(50)) return false;
    debug_pca_init_done();
    return true;
}

bool PCA9685Servo::set_position(int16_t position) {
    if (position < 0) position = 0;
    if (position > 180) position = 180;

    uint16_t pwm = angle_to_pwm(position);
    debug_servo_set(channel, position, pwm);

    if (!set_pwm(channel, 0, pwm)) return false;
    current_position = position;
    return true;
}

bool PCA9685Servo::reset() {
    uint8_t data[2] = {MODE1, 0x00};
    return i2c_handler.write(address, data, 2);
}

bool PCA9685Servo::set_pwm_freq(uint16_t freq) {
    debug_freq_start();

    float prescaleval = 25000000.0;
    prescaleval /= 4096.0;
    prescaleval /= freq;
    prescaleval -= 1.0;

    uint8_t prescale = static_cast<uint8_t>(std::floor(prescaleval + 0.5));

    uint8_t oldmode = 0;
    if (!i2c_handler.read(address, MODE1, &oldmode, 1)) return false;
    debug_mode1_before_prescale(oldmode);

    uint8_t sleep[2] = {MODE1, static_cast<uint8_t>((oldmode & ~MODE1_RESTART) | MODE1_SLEEP)};
    if (!i2c_handler.write(address, sleep, 2)) return false;

    uint8_t data[2] = {PRESCALE, prescale};
    debug_prescale(prescale);
    if (!i2c_handler.write(address, data, 2)) return false;

    uint8_t wake[2] = {MODE1, static_cast<uint8_t>((oldmode & ~MODE1_SLEEP) | MODE1_AI)};
    if (!i2c_handler.write(address, wake, 2)) return false;
    i2c_handler.delay_ms(5);

    uint8_t restart[2] = {MODE1, static_cast<uint8_t>(wake[1] | MODE1_RESTART)};
    if (!i2c_handler.write(address, restart, 2)) return false;
    debug_freq_done();
    return true;
}

bool PCA9685Servo::set_pwm(uint8_t ch, uint16_t on, uint16_t off) {
    debug_pwm_write(ch, on, off);

    uint8_t reg = LED0_ON_L + 4 * ch;
    uint8_t data[5] = {
        reg,
        static_cast<uint8_t>(on & 0xFF),
        static_cast<uint8_t>(on >> 8),
        static_cast<uint8_t>(off & 0xFF),
        static_cast<uint8_t>(off >> 8)
    };
    return i2c_handler.write(address, data, 5);
}

uint16_t PCA9685Servo::angle_to_pwm(int16_t angle) {
    const uint16_t min_pwm = 102;
    const uint16_t max_pwm = 512;
    return min_pwm + ((max_pwm - min_pwm) * angle / 180);
}
