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
- `ICM42688Driver` obtains a sample, records a 64-bit microsecond timestamp, retains raw counts, and converts measurements to g and dps. It performs neither filtering nor angle calculation.
- `IMUCalibration` accepts 50 valid warm-up samples, then averages 500 valid gyro samples and subtracts the stationary gyro bias. It preserves the accelerometer gravity vector because an arbitrary stationary pose cannot provide a valid three-axis accelerometer bias estimate. Failed reads do not advance either counter. With uninterrupted valid reads, valid sample 550 is the first calibrated row.
- `OrientationEstimator` runs after calibration. It calculates accelerometer-only and complementary-fused gravity-referenced Roll/Pitch independently, initializes from the accelerometer angle, and rejects zero, negative, non-finite, or over-0.1-second `dt` values.
- Diagnostics are compile-time controlled. The current default (`IMU_DIAGNOSTICS_ENABLED=1`) enables them; setting the macro to `0` removes them from the update path. When enabled, diagnostics emits one header after successful initialization and one 23-column row for each valid update.

```text
timestamp_us, dt_s, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz,
cal_ax_g, cal_ay_g, cal_az_g, cal_gx_dps, cal_gy_dps, cal_gz_dps,
accel_roll_deg, accel_pitch_deg, fused_roll_deg, fused_pitch_deg,
calibrated, raw_valid, calibrated_valid, estimator_initialized, estimator_valid
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

## Current Limitations

Accelerometer-only Roll/Pitch are gravity-referenced estimates:

```text
roll  = atan2(ay, az)
pitch = atan2(-ax, sqrt(ay² + az²))
```

They are appropriate for stationary conditions and slow movement, but linear acceleration and fast motion can look like tilt. Complementary-fused Roll/Pitch reduce that limitation by integrating calibrated gyro X/Y rates, but they still have no yaw estimate and require hardware validation before control-loop use. Startup-relative or user-zeroed coordinates must be a separate post-estimator reference-offset layer. Migration to a BNO085/BNO086 remains an open future design decision.
