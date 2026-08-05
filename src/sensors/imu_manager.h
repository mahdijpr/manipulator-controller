#pragma once

#include "drivers/imu/imu_driver.h"
#include "calibration/imu_calibration.h"

class IMUManager
{
public:
    bool begin();
    bool update();

    const IMUData& getData() const;

private:
    ICM42688Driver driver_;
    IMUCalibration calibration_;
    IMUData data_;
};