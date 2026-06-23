#ifdef STM
#ifndef BUZZER_STM_H
#define BUZZER_STM_H

#include "buzzer.h"

#ifdef F4
#include "stm32f4xx_hal.h"
#elif F0
#include "stm32f0xx_hal.h"
#endif

class Buzzer_STM : public Buzzer {
public:
    explicit Buzzer_STM(TIM_HandleTypeDef* htim, uint32_t channel)
        : Buzzer(channel), _htim(htim) {}

    void init() override;
    void beep(uint32_t frequency, uint32_t duration_ms) override;

    void off() override;

    void play_startup() override;
    void play_error() override;
    void play_tone(uint32_t frequency) override;

    uint32_t last_status() const { return _last_status; }
    uint32_t last_error() const { return _last_error; }

private:
    void set_frequency(uint32_t frequency);

    TIM_HandleTypeDef* _htim;

    uint32_t _last_status{0};
    uint32_t _last_error{0};

    static constexpr uint32_t TIMER_TICK_HZ = 1000000U;
};

#endif // BUZZER_STM_H
#endif // STM