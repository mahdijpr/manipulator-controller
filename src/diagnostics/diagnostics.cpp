#include "diagnostics.h"

#include <Arduino.h>

#include "common/config.h"

void Diagnostics::begin()
{
#if IMU_DIAGNOSTICS_ENABLED
    if (headerPrinted_)
        return;

    Serial.println(
        "timestamp_us,dt_s,raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz,"
        "cal_ax_g,cal_ay_g,cal_az_g,cal_gx_dps,cal_gy_dps,cal_gz_dps,"
        "accel_roll_deg,accel_pitch_deg,fused_roll_deg,fused_pitch_deg,"
        "calibrated,raw_valid,calibrated_valid,estimator_initialized,estimator_valid"
    );
    headerPrinted_ = true;
#endif
}

void Diagnostics::printIMU(const IMUData& data)
{
#if IMU_DIAGNOSTICS_ENABLED
    if (!data.raw.valid)
        return;

    Serial.printf(
        "%llu,%.6f,"
        "%d,%d,%d,%d,%d,%d,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,%.3f,"
        "%u,%u,%u,%u,%u\n",
        static_cast<unsigned long long>(data.raw.timestampUs),
        data.raw.dtSeconds,
        data.raw.rawAccelX,
        data.raw.rawAccelY,
        data.raw.rawAccelZ,
        data.raw.rawGyroX,
        data.raw.rawGyroY,
        data.raw.rawGyroZ,
        data.calibrated.accelXG,
        data.calibrated.accelYG,
        data.calibrated.accelZG,
        data.calibrated.gyroXDegS,
        data.calibrated.gyroYDegS,
        data.calibrated.gyroZDegS,
        data.orientation.accelRollDeg,
        data.orientation.accelPitchDeg,
        data.orientation.fusedRollDeg,
        data.orientation.fusedPitchDeg,
        static_cast<unsigned int>(data.calibrated.calibrationComplete),
        static_cast<unsigned int>(data.raw.valid),
        static_cast<unsigned int>(data.calibrated.valid),
        static_cast<unsigned int>(data.orientation.initialized),
        static_cast<unsigned int>(data.orientation.valid)
    );
#else
    (void)data;
#endif
}
