#pragma once

#include "common/types.h"

class IMUCalibration
{
public:

    bool begin();

    bool update(const IMURawMeasurements& input);

    const IMUCalibratedMeasurements& getData() const;

    bool isCalibrated() const;


private:

    IMUCalibratedMeasurements data_;

    bool calibrated_ = false;


    // Zero-based counts of successful processed samples in each startup stage.
    uint16_t warmupSampleCounter_ = 0;
    uint16_t sampleCounter_ = 0;


    float gxSum_ = 0.0f;
    float gySum_ = 0.0f;
    float gzSum_ = 0.0f;


    float gxOffset_ = 0.0f;
    float gyOffset_ = 0.0f;
    float gzOffset_ = 0.0f;
};
