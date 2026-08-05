#include "imu_calibration.h"

#include "common/config.h"

bool IMUCalibration::begin()
{
    data_ = IMUCalibratedMeasurements{};
    calibrated_ = false;
    warmupSampleCounter_ = 0;
    sampleCounter_ = 0;

    gxSum_ = 0.0f;
    gySum_ = 0.0f;
    gzSum_ = 0.0f;

    gxOffset_ = 0.0f;
    gyOffset_ = 0.0f;
    gzOffset_ = 0.0f;

    return true;
}

bool IMUCalibration::update(const IMURawMeasurements& input)
{
    data_.accelXG = input.accelXG;
    data_.accelYG = input.accelYG;
    data_.accelZG = input.accelZG;
    data_.gyroXDegS = input.gyroXDegS;
    data_.gyroYDegS = input.gyroYDegS;
    data_.gyroZDegS = input.gyroZDegS;
    data_.calibrationComplete = calibrated_;
    data_.valid = input.valid;

    // IMUManager normally calls this only after a successful driver update,
    // but preserve the successful-sample counter contract defensively.
    if (!input.valid)
        return false;

    if (!calibrated_)
    {
        // Warm-up samples are visible as valid, uncalibrated diagnostics rows,
        // but never affect the offset sums or calibration counter.
        if (warmupSampleCounter_ < IMU_CALIBRATION_WARMUP_SAMPLES)
        {
            ++warmupSampleCounter_;
            return true;
        }

        gxSum_ += input.gyroXDegS;
        gySum_ += input.gyroYDegS;
        gzSum_ += input.gyroZDegS;
        ++sampleCounter_;

        if (sampleCounter_ < IMU_CALIBRATION_SAMPLE_COUNT)
            return true;

        gxOffset_ = gxSum_ / IMU_CALIBRATION_SAMPLE_COUNT;
        gyOffset_ = gySum_ / IMU_CALIBRATION_SAMPLE_COUNT;
        gzOffset_ = gzSum_ / IMU_CALIBRATION_SAMPLE_COUNT;

        calibrated_ = true;
        data_.calibrationComplete = true;
    }

    // With an arbitrary stationary startup pose, a single mean accelerometer
    // vector includes gravity and cannot distinguish it from sensor bias.
    // Preserve that vector for gravity-referenced Roll/Pitch estimation.
    data_.gyroXDegS -= gxOffset_;
    data_.gyroYDegS -= gyOffset_;
    data_.gyroZDegS -= gzOffset_;

    return true;
}

const IMUCalibratedMeasurements& IMUCalibration::getData() const
{
    return data_;
}

bool IMUCalibration::isCalibrated() const
{
    return calibrated_;
}
