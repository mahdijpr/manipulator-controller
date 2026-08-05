#include "complementary_filter.h"

#include <math.h>

namespace
{
constexpr float kRadiansToDegrees = 57.2957795131f;
}

OrientationEstimator::OrientationEstimator(float timeConstantSeconds, float maximumDtSeconds)
    : timeConstantSeconds_(timeConstantSeconds), maximumDtSeconds_(maximumDtSeconds)
{
}

void OrientationEstimator::reset()
{
    output_ = OrientationEstimate{};
}

bool OrientationEstimator::update(
    const IMUCalibratedMeasurements& measurements,
    float dtSeconds
)
{
    output_.valid = false;

    if (!measurements.valid || !measurements.calibrationComplete ||
        !isfinite(measurements.accelXG) || !isfinite(measurements.accelYG) ||
        !isfinite(measurements.accelZG) || !isfinite(measurements.gyroXDegS) ||
        !isfinite(measurements.gyroYDegS))
    {
        return false;
    }

    output_.accelRollDeg = atan2f(measurements.accelYG, measurements.accelZG) * kRadiansToDegrees;
    output_.accelPitchDeg = atan2f(
        -measurements.accelXG,
        sqrtf(measurements.accelYG * measurements.accelYG + measurements.accelZG * measurements.accelZG)
    ) * kRadiansToDegrees;

    if (!isfinite(output_.accelRollDeg) || !isfinite(output_.accelPitchDeg))
        return false;

    if (!isfinite(dtSeconds) || dtSeconds <= 0.0f || dtSeconds > maximumDtSeconds_ ||
        !isfinite(timeConstantSeconds_) || timeConstantSeconds_ <= 0.0f)
    {
        return false;
    }

    // The first calibrated sample establishes the gravity-referenced attitude.
    if (!output_.initialized)
    {
        output_.fusedRollDeg = output_.accelRollDeg;
        output_.fusedPitchDeg = output_.accelPitchDeg;
        output_.initialized = true;
        output_.valid = true;
        return true;
    }

    const float alpha = timeConstantSeconds_ / (timeConstantSeconds_ + dtSeconds);

    output_.fusedRollDeg = alpha * (output_.fusedRollDeg + measurements.gyroXDegS * dtSeconds)
                           + (1.0f - alpha) * output_.accelRollDeg;
    output_.fusedPitchDeg = alpha * (output_.fusedPitchDeg + measurements.gyroYDegS * dtSeconds)
                            + (1.0f - alpha) * output_.accelPitchDeg;
    output_.valid = isfinite(output_.fusedRollDeg) && isfinite(output_.fusedPitchDeg);
    return output_.valid;
}

bool OrientationEstimator::isInitialized() const
{
    return output_.initialized;
}

bool OrientationEstimator::isValid() const
{
    return output_.valid;
}

const OrientationEstimate& OrientationEstimator::getOutput() const
{
    return output_;
}
