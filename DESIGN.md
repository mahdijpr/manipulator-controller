# Firmware Design

## Scope and Priority 1 Completion

As of 2026-08-05, Priority 1 is complete: the project reliably acquires and processes ICM42688 data on an ESP32-S3, performs startup calibration, emits optional CSV diagnostics, and has been validated at 50 Hz. It remains an IMU-processing prototype for a future manipulator controller; it has no implemented control, actuator, communication, or yaw estimation. Complementary Roll/Pitch fusion is implemented separately.

The PlatformIO environment targets `esp32-s3-devkitc-1` with Arduino and GNU C++17. Firmware code directly uses Arduino `Serial`, `Wire`, and `delay`, ESP32 `esp_timer`, and the external Arduino ICM42688 library. These dependencies mean the firmware is not currently hardware-independent and is not directly STM32-portable.

## Initialization and Runtime Architecture

```text
setup()
  -> Serial.begin(921600)
  -> Application::begin()
      -> IMUManager::begin()
          -> ICM42688Driver::begin()
              -> Wire.begin(GPIO 12, GPIO 13)
              -> Wire.setClock(400 kHz)
              -> ICM42688::begin() at 0x68
              -> accel ±4 g / gyro ±250 dps, both 100 Hz ODR
          -> IMUCalibration::begin()
          -> OrientationEstimator::reset()
      -> Diagnostics::begin() only when IMU_DIAGNOSTICS_ENABLED is true
```

An initialization failure prints an error and remains in an infinite delay loop. `IMU_DIAGNOSTICS_ENABLED` defaults to `1` in `common/config.h`, so diagnostics are enabled in the current default build, but this is not permanent: defining it as `0` compiles diagnostics out of the update path.

```text
loop()
  -> Application::update()
      -> wait until absolute deadline, if early
      -> one IMUManager::update() when due
          -> ICM42688Driver::update()
              -> ICM42688::getAGT()
              -> timestamp successful read (uint64_t microseconds)
              -> raw counts and library-converted acceleration/gyro
              -> dt from consecutive successful timestamps
          -> IMUCalibration::update()
              -> warm-up, calibrate gyro offsets, or subtract gyro offsets
          -> OrientationEstimator::update() after calibration completes
              -> calculate accelerometer Roll/Pitch and complementary fusion
      -> Diagnostics::printIMU(final data), when enabled and update succeeded
      -> advance to the first future absolute deadline
```

The schedule uses `IMU_SAMPLE_PERIOD_US = 20000` (50 Hz). The first read is due immediately after initialization. Deadlines remain anchored to the original periodic sequence; if the loop is late, skipped periods are discarded instead of creating catch-up reads. `PlatformMicros()` returns a 64-bit `esp_timer_get_time()` value. A failed `getAGT()` produces no valid diagnostic row and does not advance calibration counters, while the scheduler still advances its deadline sequence for that due period.

## Processing and Calibration Order

The order is intentionally current and concrete:

```text
ICM42688 I2C read
  -> raw counts + converted acceleration/gyro
  -> IMUCalibration gyro offsets / calibrated physical measurements
  -> OrientationEstimator accelerometer-only + fused Roll/Pitch
  -> optional Diagnostics CSV
```

`ICM42688Driver` only acquires raw counts and library-converted physical units. `IMUData` explicitly carries `raw` measurements, `calibrated` measurements, and an `orientation` estimate. `IMUCalibration` passes through accelerometer values because an arbitrary stationary startup pose cannot separate accelerometer bias from gravity; it averages stationary gyro rates after 50 valid warm-up samples, then subtracts those offsets. The first calibrated valid sample is number 550 when every read succeeds. Failed reads advance neither stage.

`OrientationEstimator` initializes fused Roll/Pitch from its gravity-referenced accelerometer angles on the first calibrated sample with an accepted `dt`. Later samples integrate calibrated X/Y gyro rates and fuse them with `alpha = tau / (tau + dt)`. It rejects zero, negative, non-finite, or greater-than-0.1-second `dt` values: the fused state remains intact and `orientation.valid` is false for that row. Accelerometer-only and fused angles remain distinct. Startup-relative or user-zeroed angles belong in a separate post-estimator reference layer if needed later.

Diagnostics is a passive final-data observer. In a diagnostics-enabled build, it prints exactly one header after initialization and one row per valid update with these 23 fields:

```text
timestamp_us, dt_s, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz,
cal_ax_g, cal_ay_g, cal_az_g, cal_gx_dps, cal_gy_dps, cal_gz_dps,
accel_roll_deg, accel_pitch_deg, fused_roll_deg, fused_pitch_deg,
calibrated, raw_valid, calibrated_valid, estimator_initialized, estimator_valid
```

## Angle Estimation

The estimator calculates gravity-referenced accelerometer angles:

```text
roll  = atan2(ay, az)
pitch = atan2(-ax, sqrt(ay² + az²))
```

It separately publishes complementary-fused Roll/Pitch. This improves dynamic tilt estimation but linear acceleration can still corrupt the accelerometer correction, and there is no yaw estimate. Hardware validation remains necessary before these outputs are used in a moving-axis control loop. PID/PD control, motor and actuator interfaces, communication stacks, and STM32 support remain future work.

## Priority 1 Validation Record

The final long timing capture contained more than 31,000 samples over approximately 622.58 s at 50.000 Hz, with approximately 9 µs timing jitter and zero timing gaps, duplicate/non-increasing timestamps, or estimated missing samples.

Three independent stationary post-calibration tests each maintained 50.000 Hz, with approximately 7–10 µs jitter and no timing gaps, duplicate timestamps, or missing samples. Roll and pitch standard deviations were approximately 0.029° and 0.032°. The observed `raw_gz` peak-to-peak ranges were approximately 22, 23, and 24 counts; `gz_dps` standard deviation was approximately 0.0054–0.0060 dps. Calculated drift was negligible within noise.

The earlier large `raw_gz` excursions were absent from all stationary tests. They may have resulted from fast motion, impact, Z-axis rotation, or cable movement during motion testing, but the exact cause was not proven. No outlier detector is currently justified: there is no evidence of persistent I2C or sensor failure, and a simplistic detector could reject genuine rapid motion.

Motion tests retained the 50 Hz rate, captured positive and negative roll/pitch movement, returned roll to its initial position within approximately 0.08° and pitch within approximately 0.10°, and showed acceptable slow-motion axis separation.
