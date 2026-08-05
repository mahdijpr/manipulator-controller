#pragma once

#include <stdint.h>

// A successful driver read. Counts and their library-converted physical units
// remain uncalibrated at this stage.
struct IMURawMeasurements
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

    // Library-converted sensor measurements.
    float accelXG = 0.0f;
    float accelYG = 0.0f;
    float accelZG = 0.0f;

    float gyroXDegS = 0.0f;
    float gyroYDegS = 0.0f;
    float gyroZDegS = 0.0f;

    bool valid = false;
};

// Physical measurements after startup calibration. A single arbitrary
// stationary pose cannot determine accelerometer bias without conflating it
// with gravity, so accelerometer values are passed through unless a known
// calibration pose is introduced later. Gyro values are bias-corrected.
struct IMUCalibratedMeasurements
{
    float accelXG = 0.0f;
    float accelYG = 0.0f;
    float accelZG = 0.0f;

    float gyroXDegS = 0.0f;
    float gyroYDegS = 0.0f;
    float gyroZDegS = 0.0f;

    bool calibrationComplete = false;
    bool valid = false;
};

// Orientation-estimator outputs. Accelerometer-only angles are kept distinct
// from the complementary-fused roll and pitch values.
struct OrientationEstimate
{
    float accelRollDeg = 0.0f;
    float accelPitchDeg = 0.0f;
    float fusedRollDeg = 0.0f;
    float fusedPitchDeg = 0.0f;

    bool initialized = false;
    bool valid = false;
};

// The complete runtime data flow: driver -> calibration -> orientation.
struct IMUData
{
    IMURawMeasurements raw;
    IMUCalibratedMeasurements calibrated;
    OrientationEstimate orientation;
};
