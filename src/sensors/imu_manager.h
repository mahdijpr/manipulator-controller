#pragma once

#include "drivers/imu/imu_driver.h"
#include "calibration/imu_calibration.h"
#include "filters/complementary_filter.h"

class IMUManager
{
public:
    IMUManager();

    bool begin();
    bool update();

    const IMUData& getData() const;

private:
    ICM42688Driver driver_;
    IMUCalibration calibration_;
    ComplementaryFilter attitudeFilter_;
    IMUData data_;
};
