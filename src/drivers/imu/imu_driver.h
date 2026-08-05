#pragma once

#include "common/types.h"
#include "imu_interface.h"
#include "filters/low_pass_filter.h"



class ICM42688Driver : public IIMUDriver
{
public:


    ICM42688Driver();
    
    bool begin();

    bool update();

    const IMUData& getData() const;

private:

    IMUData data_;

    uint64_t lastTimestampUs_ = 0;

    LowPassFilter gyroXFilter;
    LowPassFilter gyroYFilter;
    LowPassFilter gyroZFilter;
};
