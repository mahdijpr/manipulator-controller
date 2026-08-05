// Host-side checks: compile and run with
// g++ -std=c++17 -Isrc test/host_imu_validation.cpp src/calibration/imu_calibration.cpp
#include <assert.h>

#include "app/periodic_deadline.h"
#include "calibration/imu_calibration.h"
#include "common/config.h"

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
    return 0;
}
