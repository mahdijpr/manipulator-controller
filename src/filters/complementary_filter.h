#pragma once

#include "common/types.h"

class OrientationEstimator
{
public:
    OrientationEstimator(float timeConstantSeconds, float maximumDtSeconds);

    void reset();
    bool update(
        const IMUCalibratedMeasurements& measurements,
        float dtSeconds
    );

    bool isInitialized() const;
    bool isValid() const;
    const OrientationEstimate& getOutput() const;

private:
    float timeConstantSeconds_;
    float maximumDtSeconds_;
    OrientationEstimate output_;
};
