#pragma once

#include "common/types.h"

class IMUCalibration
{
public:

    bool begin();

    bool update(const IMUData& input);

    const IMUData& getData() const;

    bool isCalibrated() const;


private:

    IMUData data_;

    bool calibrated_ = false;


    // Zero-based counts of successful processed samples in each startup stage.
    uint16_t warmupSampleCounter_ = 0;
    uint16_t sampleCounter_ = 0;


    float rollSum_ = 0.0f;
    float pitchSum_ = 0.0f;


    float gxSum_ = 0.0f;
    float gySum_ = 0.0f;
    float gzSum_ = 0.0f;


    float rollOffset_ = 0.0f;
    float pitchOffset_ = 0.0f;


    float gxOffset_ = 0.0f;
    float gyOffset_ = 0.0f;
    float gzOffset_ = 0.0f;
};
