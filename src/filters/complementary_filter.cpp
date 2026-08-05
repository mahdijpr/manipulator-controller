#include "complementary_filter.h"

ComplementaryFilter::ComplementaryFilter(float timeConstantSeconds)
    : timeConstantSeconds_(timeConstantSeconds > 0.0f ? timeConstantSeconds : 0.0f)
{
}

void ComplementaryFilter::reset()
{
    rollDeg_ = 0.0f;
    pitchDeg_ = 0.0f;
    initialized_ = false;
}

void ComplementaryFilter::initialize(float rollDeg, float pitchDeg)
{
    rollDeg_ = rollDeg;
    pitchDeg_ = pitchDeg;
    initialized_ = true;
}

void ComplementaryFilter::update(
    float accelRollDeg,
    float accelPitchDeg,
    float gyroXDegS,
    float gyroYDegS,
    float dtSeconds
)
{
    if (!initialized_)
    {
        initialize(accelRollDeg, accelPitchDeg);
        return;
    }

    if (dtSeconds <= 0.0f)
        return;

    const float alpha = timeConstantSeconds_ / (timeConstantSeconds_ + dtSeconds);

    rollDeg_ = alpha * (rollDeg_ + gyroXDegS * dtSeconds)
             + (1.0f - alpha) * accelRollDeg;
    pitchDeg_ = alpha * (pitchDeg_ + gyroYDegS * dtSeconds)
              + (1.0f - alpha) * accelPitchDeg;
}

bool ComplementaryFilter::isInitialized() const
{
    return initialized_;
}

float ComplementaryFilter::getRollDeg() const
{
    return rollDeg_;
}

float ComplementaryFilter::getPitchDeg() const
{
    return pitchDeg_;
}
