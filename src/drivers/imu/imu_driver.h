#pragma once

#include "common/types.h"
#include "imu_interface.h"

class ICM42688Driver : public IIMUDriver
{
public:
    bool begin();

    bool update();

    const IMURawMeasurements& getData() const;

private:

    IMURawMeasurements data_;

    uint64_t lastTimestampUs_ = 0;
};
