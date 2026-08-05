#pragma once

#include <stdint.h>

uint32_t PlatformMillis();
uint64_t PlatformMicros();
void PlatformDelayMilliseconds(uint32_t delayMs);
