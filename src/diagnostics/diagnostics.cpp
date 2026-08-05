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
        "roll_deg,pitch_deg,gx_dps,gy_dps,gz_dps,calibrated,valid"
    );
    headerPrinted_ = true;
#endif
}

void Diagnostics::printIMU(const IMUData& data)
{
#if IMU_DIAGNOSTICS_ENABLED
    if (!data.valid)
        return;

    Serial.printf(
        "%llu,%.6f,"
        "%d,%d,%d,%d,%d,%d,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,"
        "%u,%u\n",
        static_cast<unsigned long long>(data.timestampUs),
        data.dtSeconds,
        data.rawAccelX,
        data.rawAccelY,
        data.rawAccelZ,
        data.rawGyroX,
        data.rawGyroY,
        data.rawGyroZ,
        data.rollDeg,
        data.pitchDeg,
        data.gyroXDegS,
        data.gyroYDegS,
        data.gyroZDegS,
        static_cast<unsigned int>(data.calibrated),
        static_cast<unsigned int>(data.valid)
    );
#else
    (void)data;
#endif
}
