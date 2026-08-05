#include "imu_calibration.h"

#include "common/config.h"

bool IMUCalibration::begin()
{
    data_ = IMUData{};
    calibrated_ = false;
    warmupSampleCounter_ = 0;
    sampleCounter_ = 0;

    rollSum_ = 0.0f;
    pitchSum_ = 0.0f;
    gxSum_ = 0.0f;
    gySum_ = 0.0f;
    gzSum_ = 0.0f;

    rollOffset_ = 0.0f;
    pitchOffset_ = 0.0f;
    gxOffset_ = 0.0f;
    gyOffset_ = 0.0f;
    gzOffset_ = 0.0f;

    return true;
}

bool IMUCalibration::update(const IMUData& input)
{
    data_ = input;
    data_.calibrated = calibrated_;

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

        rollSum_ += input.rollDeg;
        pitchSum_ += input.pitchDeg;
        gxSum_ += input.gyroXDegS;
        gySum_ += input.gyroYDegS;
        gzSum_ += input.gyroZDegS;
        ++sampleCounter_;

        if (sampleCounter_ < IMU_CALIBRATION_SAMPLE_COUNT)
            return true;

        rollOffset_ = rollSum_ / IMU_CALIBRATION_SAMPLE_COUNT;
        pitchOffset_ = pitchSum_ / IMU_CALIBRATION_SAMPLE_COUNT;
        gxOffset_ = gxSum_ / IMU_CALIBRATION_SAMPLE_COUNT;
        gyOffset_ = gySum_ / IMU_CALIBRATION_SAMPLE_COUNT;
        gzOffset_ = gzSum_ / IMU_CALIBRATION_SAMPLE_COUNT;

        calibrated_ = true;
        data_.calibrated = true;
    }

    data_.rollDeg -= rollOffset_;
    data_.pitchDeg -= pitchOffset_;
    data_.gyroXDegS -= gxOffset_;
    data_.gyroYDegS -= gyOffset_;
    data_.gyroZDegS -= gzOffset_;

    return true;
}

const IMUData& IMUCalibration::getData() const
{
    return data_;
}

bool IMUCalibration::isCalibrated() const
{
    return calibrated_;
}
