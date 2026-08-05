#include "imu_manager.h"

#include "common/config.h"

IMUManager::IMUManager()
    : attitudeFilter_(IMU_COMPLEMENTARY_FILTER_TIME_CONSTANT_S)
{
}

bool IMUManager::begin()
{
    if (!driver_.begin())
        return false;

    calibration_.begin();
    attitudeFilter_.reset();

    return true;
}

bool IMUManager::update()
{
    if (!driver_.update())
        return false;

    calibration_.update(driver_.getData());
    data_ = calibration_.getData();

    // Startup calibration establishes the accelerometer-angle and gyro-rate
    // zero references. Begin fusion from the first calibrated angle so the
    // filter cannot integrate uncalibrated gyro bias during startup.
    if (data_.calibrated)
    {
        if (!attitudeFilter_.isInitialized())
        {
            attitudeFilter_.initialize(data_.rollDeg, data_.pitchDeg);
        }
        else
        {
            attitudeFilter_.update(
                data_.rollDeg,
                data_.pitchDeg,
                data_.gyroXDegS,
                data_.gyroYDegS,
                data_.dtSeconds
            );
        }

        data_.rollDeg = attitudeFilter_.getRollDeg();
        data_.pitchDeg = attitudeFilter_.getPitchDeg();
    }

    return true;
}

const IMUData& IMUManager::getData() const
{
    return data_;
}
