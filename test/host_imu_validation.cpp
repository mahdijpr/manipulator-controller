// Host-side checks: compile and run with
// g++ -std=c++17 -Isrc test/host_imu_validation.cpp
//     src/calibration/imu_calibration.cpp src/filters/complementary_filter.cpp
#include <assert.h>
#include <math.h>
#include <limits>

#include "app/periodic_deadline.h"
#include "calibration/imu_calibration.h"
#include "common/config.h"
#include "filters/complementary_filter.h"

static bool Near(float actual, float expected, float tolerance = 0.0001f)
{
    return fabsf(actual - expected) <= tolerance;
}

static IMUCalibratedMeasurements LevelMeasurements()
{
    IMUCalibratedMeasurements measurements{};
    measurements.accelZG = 1.0f;
    measurements.calibrationComplete = true;
    measurements.valid = true;
    return measurements;
}

int main()
{
    constexpr uint64_t initialDeadlineUs = 1000000U;
    assert(AdvancePeriodicDeadline(initialDeadlineUs, initialDeadlineUs, 20000U) == 1020000U);
    assert(AdvancePeriodicDeadline(initialDeadlineUs, 1019999U, 20000U) == 1020000U);
    assert(AdvancePeriodicDeadline(initialDeadlineUs, 1020001U, 20000U) == 1040000U);
    assert(AdvancePeriodicDeadline(initialDeadlineUs, 1060000U, 20000U) == 1080000U);

    // Startup calibration estimates stationary gyro bias but preserves an
    // arbitrary gravity vector rather than treating it as accelerometer bias.
    IMUCalibration calibration;
    assert(calibration.begin());
    IMURawMeasurements sample{};
    sample.valid = true;
    sample.accelXG = 0.2f;
    sample.accelYG = -0.3f;
    sample.accelZG = 0.93f;
    sample.gyroXDegS = 1.0f;
    sample.gyroYDegS = 2.0f;
    sample.gyroZDegS = 3.0f;

    IMURawMeasurements failedRead = sample;
    failedRead.valid = false;
    assert(!calibration.update(failedRead));
    assert(!calibration.isCalibrated());

    for (uint16_t row = 0;
         row < IMU_CALIBRATION_WARMUP_SAMPLES + IMU_CALIBRATION_SAMPLE_COUNT - 1U;
         ++row)
    {
        assert(calibration.update(sample));
        assert(!calibration.isCalibrated());
        assert(!calibration.getData().calibrationComplete);
    }

    assert(calibration.update(sample));
    assert(calibration.isCalibrated());
    const IMUCalibratedMeasurements& calibrated = calibration.getData();
    assert(calibrated.calibrationComplete);
    assert(Near(calibrated.accelXG, sample.accelXG));
    assert(Near(calibrated.accelYG, sample.accelYG));
    assert(Near(calibrated.accelZG, sample.accelZG));
    assert(Near(calibrated.gyroXDegS, 0.0f));
    assert(Near(calibrated.gyroYDegS, 0.0f));
    assert(Near(calibrated.gyroZDegS, 0.0f));

    // Initialization derives gravity-referenced accelerometer angles and
    // copies them to the first fused output without an arbitrary gyro step.
    OrientationEstimator initializationEstimator(0.5f, 0.1f);
    IMUCalibratedMeasurements tilted = LevelMeasurements();
    tilted.accelYG = 0.5f;
    tilted.accelZG = 0.8660254f;
    assert(!initializationEstimator.isInitialized());
    assert(!initializationEstimator.isValid());
    assert(!initializationEstimator.update(tilted, 0.0f));
    assert(!initializationEstimator.isInitialized());
    assert(!initializationEstimator.isValid());
    assert(initializationEstimator.update(tilted, 0.02f));
    const OrientationEstimate& initialized = initializationEstimator.getOutput();
    assert(initialized.initialized);
    assert(initialized.valid);
    assert(Near(initialized.accelRollDeg, 30.0f, 0.001f));
    assert(Near(initialized.accelPitchDeg, 0.0f, 0.001f));
    assert(Near(initialized.fusedRollDeg, initialized.accelRollDeg));

    // Gyro integration occurs before complementary correction. A very large
    // time constant makes this test effectively gyro-only for one update.
    OrientationEstimator integrationEstimator(1000.0f, 0.1f);
    IMUCalibratedMeasurements level = LevelMeasurements();
    assert(integrationEstimator.update(level, 0.02f));
    level.gyroXDegS = 10.0f;
    assert(integrationEstimator.update(level, 0.02f));
    assert(Near(integrationEstimator.getOutput().fusedRollDeg, 0.199996f, 0.0001f));

    // With tau=0.5 s and dt=0.02 s, alpha=0.961538. The accelerometer-only
    // output remains independently available while fused Roll is corrected.
    OrientationEstimator correctionEstimator(0.5f, 0.1f);
    assert(correctionEstimator.update(level, 0.02f));
    assert(correctionEstimator.update(level, 0.02f));
    const float fusedAfterGyro = correctionEstimator.getOutput().fusedRollDeg;
    assert(Near(fusedAfterGyro, 0.1923077f, 0.0001f));
    level.gyroXDegS = 0.0f;
    assert(correctionEstimator.update(level, 0.02f));
    assert(correctionEstimator.getOutput().fusedRollDeg < fusedAfterGyro);
    assert(Near(correctionEstimator.getOutput().accelRollDeg, 0.0f));

    // Stationary updates converge the fused attitude to the accelerometer.
    for (uint16_t row = 0; row < 200U; ++row)
        assert(correctionEstimator.update(level, 0.02f));
    assert(Near(correctionEstimator.getOutput().fusedRollDeg, 0.0f, 0.0001f));

    // Invalid timing intervals never integrate gyro data, leave the prior
    // fused angle intact, and mark that output invalid without deinitializing.
    const float fusedBeforeInvalidDt = correctionEstimator.getOutput().fusedRollDeg;
    const float invalidDts[] = {
        0.0f,
        -0.02f,
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::infinity(),
        0.100001f,
    };
    for (float dt : invalidDts)
    {
        assert(!correctionEstimator.update(level, dt));
        assert(correctionEstimator.isInitialized());
        assert(!correctionEstimator.isValid());
        assert(Near(correctionEstimator.getOutput().fusedRollDeg, fusedBeforeInvalidDt));
    }
    assert(correctionEstimator.update(level, 0.02f));
    assert(correctionEstimator.isValid());

    correctionEstimator.reset();
    assert(!correctionEstimator.isInitialized());
    assert(!correctionEstimator.isValid());
    assert(Near(correctionEstimator.getOutput().fusedRollDeg, 0.0f));

    return 0;
}
