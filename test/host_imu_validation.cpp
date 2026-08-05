// Host-side checks: compile and run with
// g++ -std=c++17 -Isrc test/host_imu_validation.cpp //     src/calibration/imu_calibration.cpp src/filters/complementary_filter.cpp
#include <assert.h>
#include <math.h>

#include "app/periodic_deadline.h"
#include "calibration/imu_calibration.h"
#include "common/config.h"
#include "filters/complementary_filter.h"

static bool Near(float actual, float expected, float tolerance = 0.0001f)
{
    return fabsf(actual - expected) <= tolerance;
}

int main()
{
    constexpr uint64_t initialDeadlineUs = 1000000U;
    assert(AdvancePeriodicDeadline(initialDeadlineUs, initialDeadlineUs, 20000U) == 1020000U);
    assert(AdvancePeriodicDeadline(initialDeadlineUs, 1019999U, 20000U) == 1020000U);
    assert(AdvancePeriodicDeadline(initialDeadlineUs, 1020001U, 20000U) == 1040000U);
    assert(AdvancePeriodicDeadline(initialDeadlineUs, 1060000U, 20000U) == 1080000U);

    IMUCalibration calibration;
    assert(calibration.begin());
    IMUData sample{};
    sample.valid = true;
    sample.rollDeg = 10.0f;
    sample.pitchDeg = -5.0f;
    sample.gyroXDegS = 1.0f;
    sample.gyroYDegS = 2.0f;
    sample.gyroZDegS = 3.0f;

    IMUData failedRead = sample;
    failedRead.valid = false;
    assert(!calibration.update(failedRead));
    assert(!calibration.isCalibrated());

    for (uint16_t row = 0;
         row < IMU_CALIBRATION_WARMUP_SAMPLES + IMU_CALIBRATION_SAMPLE_COUNT - 1U;
         ++row)
    {
        assert(calibration.update(sample));
        assert(!calibration.isCalibrated());
        assert(!calibration.getData().calibrated);
    }

    assert(calibration.update(sample));
    assert(calibration.isCalibrated());
    assert(calibration.getData().calibrated);

    ComplementaryFilter filter(0.5f);
    assert(!filter.isInitialized());

    // The first fused sample initializes from the accelerometer angle and does
    // not integrate an arbitrary gyro interval from before filter startup.
    filter.update(3.0f, -2.0f, 100.0f, 100.0f, 0.02f);
    assert(filter.isInitialized());
    assert(Near(filter.getRollDeg(), 3.0f));
    assert(Near(filter.getPitchDeg(), -2.0f));

    // tau=0.5 s and dt=0.02 s produce alpha=0.961538.
    filter.update(3.0f, -2.0f, 10.0f, -5.0f, 0.02f);
    assert(Near(filter.getRollDeg(), 3.1923077f));
    assert(Near(filter.getPitchDeg(), -2.0961538f));

    const float rollBeforeInvalidDt = filter.getRollDeg();
    filter.update(0.0f, 0.0f, 100.0f, 100.0f, 0.0f);
    assert(Near(filter.getRollDeg(), rollBeforeInvalidDt));

    filter.reset();
    assert(!filter.isInitialized());

    return 0;
}
