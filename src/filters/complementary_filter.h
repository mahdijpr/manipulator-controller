#pragma once

class ComplementaryFilter
{
public:
    explicit ComplementaryFilter(float timeConstantSeconds);

    void reset();
    void initialize(float rollDeg, float pitchDeg);
    void update(
        float accelRollDeg,
        float accelPitchDeg,
        float gyroXDegS,
        float gyroYDegS,
        float dtSeconds
    );

    bool isInitialized() const;
    float getRollDeg() const;
    float getPitchDeg() const;

private:
    float timeConstantSeconds_;
    float rollDeg_ = 0.0f;
    float pitchDeg_ = 0.0f;
    bool initialized_ = false;
};
