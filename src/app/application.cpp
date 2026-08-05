#include <Arduino.h>

#include "application.h"
#include "app/periodic_deadline.h"
#include "common/config.h"
#include "platform/platform_time.h"


bool Application::begin()
{
    if (!imu_.begin())
    {
        Serial.println("IMU Init Failed");

        while (true)
        {
            delay(100);
        }
    }

#if IMU_DIAGNOSTICS_ENABLED
    diagnostics_.begin();
#endif

    // The first acquisition is due immediately after initialization. Each
    // later deadline remains anchored to this periodic sequence.
    nextSampleDeadlineUs_ = PlatformMicros();

    return true;
}

void Application::update()
{
    const uint64_t nowUs = PlatformMicros();
    if (nowUs < nextSampleDeadlineUs_)
    {
        // Yield most of the remaining interval instead of busy-waiting. The
        // sub-millisecond remainder is intentionally left to the Arduino loop.
        const uint64_t remainingUs = nextSampleDeadlineUs_ - nowUs;
        if (remainingUs >= 1000U)
            PlatformDelayMilliseconds(static_cast<uint32_t>(remainingUs / 1000U));
        return;
    }

    // Exactly one read is attempted for this scheduled period, regardless of
    // whether it succeeds. If late, skipped periods are discarded rather than
    // triggering back-to-back catch-up reads.
#if IMU_DIAGNOSTICS_ENABLED
    if (imu_.update())
    {
        // Diagnostics observes the final calibrated manager output once per cycle.
        diagnostics_.printIMU(imu_.getData());
    }
#else
    imu_.update();
#endif

    nextSampleDeadlineUs_ = AdvancePeriodicDeadline(
        nextSampleDeadlineUs_,
        PlatformMicros(),
        IMU_SAMPLE_PERIOD_US
    );

}
