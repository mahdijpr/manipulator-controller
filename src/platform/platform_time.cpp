#include <Arduino.h>
#include <esp_timer.h>

uint32_t PlatformMillis()
{
    return millis();
}

uint64_t PlatformMicros()
{
    return static_cast<uint64_t>(esp_timer_get_time());
}

void PlatformDelayMilliseconds(uint32_t delayMs)
{
    delay(delayMs);
}
