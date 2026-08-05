#include "imu_manager.h"

#include "common/config.h"

IMUManager::IMUManager()
    : orientationEstimator_(
          IMU_COMPLEMENTARY_FILTER_TIME_CONSTANT_S,
          IMU_ORIENTATION_ESTIMATOR_MAX_DT_S
      )
{
}

bool IMUManager::begin()
{
    if (!driver_.begin())
        return false;

    calibration_.begin();
    orientationEstimator_.reset();
    data_ = IMUData{};

    return true;
}

bool IMUManager::update()
{
    if (!driver_.update())
        return false;

    if (!calibration_.update(driver_.getData()))
        return false;

    data_.raw = driver_.getData();
    data_.calibrated = calibration_.getData();

    // Only calibrated measurements enter the estimator. The first such sample
    // initializes fused Roll/Pitch from gravity-referenced accelerometer angles.
    if (data_.calibrated.calibrationComplete)
    {
        orientationEstimator_.update(data_.calibrated, data_.raw.dtSeconds);
    }

    data_.orientation = orientationEstimator_.getOutput();

    return true;
}

const IMUData& IMUManager::getData() const
{
    return data_;
}
