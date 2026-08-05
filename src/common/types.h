#pragma once

#include <stdint.h>

struct IMUData
{
    // Timestamp captured immediately after a successful ICM42688 sample read.
    uint64_t timestampUs = 0;
    float dtSeconds = 0.0f;

    // Unfiltered, uncalibrated ICM42688 ADC counts.
    int16_t rawAccelX = 0;
    int16_t rawAccelY = 0;
    int16_t rawAccelZ = 0;

    int16_t rawGyroX = 0;
    int16_t rawGyroY = 0;
    int16_t rawGyroZ = 0;

    // Library-converted accelerometer measurements, in g.
    float accelXG = 0.0f;
    float accelYG = 0.0f;
    float accelZG = 0.0f;

    // Final processed values: roll/pitch and filtered gyro after calibration.
    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;

    float gyroXDegS = 0.0f;
    float gyroYDegS = 0.0f;
    float gyroZDegS = 0.0f;

    bool calibrated = false;
    bool valid = false;
};
