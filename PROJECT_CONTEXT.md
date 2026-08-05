# Project Context

## Status — Priority 1 Complete (2026-08-05)

This is an ESP32-S3 IMU-processing prototype for a future manipulator controller, not a complete motion controller. Priority 1—reliable IMU acquisition, startup calibration, diagnostics, and timing validation—is complete. The firmware has no actuator, encoder, kinematics, trajectory, PID/PD/ADRC, yaw-estimation, or communication implementation.

## Current Hardware and Build

| Item | Verified implementation |
| --- | --- |
| Board / framework | ESP32-S3 DevKitC-1, PlatformIO, Arduino, GNU C++17 |
| Serial monitor / upload speed | 921600 baud |
| IMU | ICM42688 on I2C address `0x68` |
| I2C bus | SDA GPIO 12, SCL GPIO 13, `400 kHz` |
| Sensor setup | Accelerometer ±4 g at 100 Hz; gyroscope ±250 dps at 100 Hz |
| Application schedule | Absolute `20000 us` deadlines (50 Hz) using 64-bit microsecond timestamps |

The PlatformIO dependency is the Arduino ICM42688 library at `https://github.com/finani/ICM42688`; the URL is not pinned. The current firmware also depends directly on Arduino `Serial`, `Wire`, and delay APIs, plus ESP32 `esp_timer`, so it is not hardware-independent or readily portable to STM32 as-is.

## Completed Priority 1 Behavior

- `Application` schedules one IMU update per due period. When late, it advances to the next future absolute deadline and does not make catch-up bursts.
- `ICM42688Driver` obtains a sample, records a 64-bit microsecond timestamp, retains raw counts, filters gyro values, and calculates accelerometer-based roll and pitch.
- `IMUCalibration` accepts 50 valid warm-up samples, then averages 500 valid calibration samples. Failed reads do not advance either counter. With uninterrupted valid reads, valid sample 550 is the first calibrated row.
- Diagnostics are compile-time controlled. The current default (`IMU_DIAGNOSTICS_ENABLED=1`) enables them; setting the macro to `0` removes them from the update path. When enabled, diagnostics emits one header after successful initialization and one 15-column row for each valid update.

```text
timestamp_us, dt_s, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz,
roll_deg, pitch_deg, gx_dps, gy_dps, gz_dps, calibrated, valid
```

## Priority 1 Validation

| Validation | Result |
| --- | --- |
| Long timing capture | More than 31,000 samples across approximately 622.58 s |
| Sampling rate | 50.000 Hz |
| Timing jitter | Approximately 9 µs |
| Timing integrity | Zero timing gaps, duplicate/non-increasing timestamps, and estimated missing samples |
| Three stationary post-calibration tests | 50.000 Hz in every test; approximately 7–10 µs jitter; no gaps, duplicate timestamps, or missing samples |
| Stationary orientation stability | Roll standard deviation approximately 0.029°, pitch standard deviation approximately 0.032° |
| Stationary Z gyro observations | `raw_gz` peak-to-peak ranges approximately 22, 23, and 24 counts; `gz_dps` standard deviation approximately 0.0054–0.0060 dps |
| Drift | Negligible and within the measured noise level |

The large `raw_gz` excursions seen in earlier motion testing did not occur in any stationary test. There is no evidence of a persistent sensor or I2C failure, so no outlier detector has been added; a simple detector could reject legitimate fast motion.

Motion tests retained stable 50 Hz timing, captured positive and negative roll and pitch movement, and showed return to the initial position within approximately 0.08° roll and 0.10° pitch. Slow-motion axis separation was acceptable. The earlier large `raw_gz` values may have come from fast motion, impact, Z-axis rotation, or cable movement; the cause was not proven.

## Current Limitations and Priority 2

Roll and pitch are accelerometer-only estimates:

```text
roll  = atan2(ay, az)
pitch = atan2(-ax, sqrt(ay² + az²))
```

They are appropriate for stationary conditions and slow movement, but linear acceleration and fast motion can look like tilt. Gyroscope measurements are low-pass filtered and calibrated but are not fused into the angles. Therefore the current angles are not suitable for direct use in a moving-axis control loop.

Priority 2 is the implementation and validation of dynamic angle estimation and sensor fusion. The proposed next step is a complementary filter using the existing ICM42688. Migration to a BNO085/BNO086 remains an open future design decision, not a current commitment.
