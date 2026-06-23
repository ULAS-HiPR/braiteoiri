#ifndef BUZZER_H
#define BUZZER_H

#include <cstdint>
#include <cstddef>

class Buzzer {
public:
    explicit Buzzer(uint32_t channel) : _channel(channel) {}

    virtual void init() = 0;
    virtual void beep(uint32_t frequency, uint32_t duration_ms) = 0;

    virtual void off() {}

    virtual void play_startup() {}
    virtual void play_error() {}
    virtual void play_tone(uint32_t frequency) {}

    virtual ~Buzzer() = default;

protected:
    uint32_t _channel;
};

#endif // BUZZER_H