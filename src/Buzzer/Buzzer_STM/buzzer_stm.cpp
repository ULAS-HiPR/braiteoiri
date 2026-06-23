#include <Buzzer/buzzer_stm.h>

void Buzzer_STM::init()
{
    _last_status = HAL_TIM_PWM_Start(_htim, _channel);
    off();
}

void Buzzer_STM::off()
{
    __HAL_TIM_SET_COMPARE(_htim, _channel, 0);
}

void Buzzer_STM::set_frequency(uint32_t frequency)
{
    if (frequency == 0U) {
        off();
        return;
    }

    uint32_t period = TIMER_TICK_HZ / frequency;

    if (period < 2U) period = 2U;
    if (period > 0xFFFFU) period = 0xFFFFU;

    __HAL_TIM_SET_AUTORELOAD(_htim, period - 1U);
    __HAL_TIM_SET_COMPARE(_htim, _channel, period / 2U);
    __HAL_TIM_SET_COUNTER(_htim, 0);
}

void Buzzer_STM::beep(uint32_t frequency, uint32_t duration_ms)
{
    set_frequency(frequency);

    if (frequency == 0U) {
        HAL_Delay(duration_ms);
        return;
    }

    if (duration_ms > 30U) {
        HAL_Delay(duration_ms - 20U);
        off();
        HAL_Delay(20U);
    } else {
        HAL_Delay(duration_ms);
        off();
    }
}

void Buzzer_STM::play_tone(uint32_t frequency)
{
    set_frequency(frequency);
}

void Buzzer_STM::play_startup()
{
    beep(2400, 80);
    HAL_Delay(100);
    beep(2400, 80);
    HAL_Delay(100);
    beep(2400, 80);
}

void Buzzer_STM::play_error()
{
    beep(400, 200);
    HAL_Delay(100);
    beep(200, 400);
}