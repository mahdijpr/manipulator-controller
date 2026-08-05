# Project Context

## Overview

This is an **ESP32-S3 based IMU acquisition and processing prototype for a future manipulator controller**. It is not yet a completed motion controller.

The current firmware brings up one ICM42688 IMU, acquires measurements, applies a gyro low-pass filter, derives roll and pitch from the accelerometer, and performs startup offset calibration. The resulting data is available to the application layer but is not yet used to control hardware.

## Current Status

Implemented now:

- ESP32-S3 DevKitC-1 PlatformIO/Arduino project setup.
- ICM42688 initialization and I2C communication.
- Raw accelerometer and gyroscope capture.
- Library-provided acceleration values in g and gyroscope values in dps.
- First-order low-pass filtering of the three gyro axes.
- Accelerometer-derived roll and pitch calculations.
- Microsecond acquisition timestamp and elapsed-time (`dt`) capture.
- Startup averaging calibration for roll, pitch, and filtered gyro offsets.
- Compile-time controlled, standardized CSV diagnostics and an offline Python log-analysis script.

Not active or not implemented now:

- Motor/servo drivers, encoders, kinematics, trajectory generation, or joint control.
- PID, ADRC, sensor fusion, yaw estimation, or communication protocols.
- Persistent calibration, calibration validation, runtime recovery, or STM32 support.

## Current Hardware and Build Environment

| Item | Current implementation |
| --- | --- |
| MCU / board | ESP32-S3 DevKitC-1 |
| Build system | PlatformIO |
| Framework | Arduino |
| Language mode | GNU C++17 |
| Serial speed | 921600 baud |
| IMU | ICM42688 |
| IMU bus | I2C, 400 kHz |
| IMU address | `0x68` |
| I2C pins | SDA GPIO 12, SCL GPIO 13 |

The declared third-party firmware dependency is the `finani/ICM42688` GitHub library. Its URL is currently unpinned, so a fresh dependency resolution can change over time.

## Current Sensor Configuration

| Sensor channel | Full-scale range | Configured ODR |
| --- | --- | --- |
| Accelerometer | +/-4 g | 100 Hz |
| Gyroscope | +/-250 dps | 100 Hz |

The application schedules IMU acquisition at 50 Hz with `IMU_SAMPLE_PERIOD_US = 20,000`. It uses the ESP32's 64-bit monotonic microsecond time, retains an absolute deadline sequence, and yields only while waiting before a deadline. It does not apply a fixed delay after acquisition and processing. When late, it performs one acquisition attempt and advances the deadline by whole periods to a future deadline, without catch-up bursts. Physical-board capture is still needed to measure the achieved rate and jitter.

## Current Software Architecture

```text
main.cpp
  -> Application
      -> IMUManager
          -> ICM42688Driver
              -> Arduino Wire + ICM42688 library + I2C hardware
              -> three gyro LowPassFilter instances
          -> IMUCalibration
      -> Diagnostics (passively observes final IMUManager data when enabled)
```

At runtime, the driver reads the sensor, timestamps the acquired sample in microseconds, stores raw and converted values, filters gyro values, and calculates roll/pitch. `IMUManager` then passes that data through `IMUCalibration` and returns the corrected data to `Application`, which emits one CSV row through `Diagnostics` for each successful update when diagnostics are enabled.

`IIMUDriver` is implemented by `ICM42688Driver`, but `IMUManager` currently owns the concrete driver type. The interface is therefore present but is not yet the active substitution boundary.

## Current Limitations

- Startup calibration assumes the IMU is stationary for the 50 successful warm-up samples (about one second at 50 Hz) plus 500 successful calibration samples; this is not checked in code. Warm-up samples run filtering and appear as valid, uncalibrated CSV rows but do not contribute to offsets. With no failed reads, the first calibrated CSV row is valid row 550 (one-based). Failed reads advance neither counter.
- Only gyro data is filtered. Roll and pitch come directly from accelerometer data; no yaw or sensor fusion exists.
- An initialization failure blocks indefinitely. Runtime read failures produce no CSV row and have no recovery action.
- Arduino APIs are used in several modules, so the present code is not yet hardware independent.

## Future Roadmap

The following are **future work**, not current capabilities:

- Build the manipulator controller: actuator drivers, encoders, joint control, and trajectory handling.
- Add control algorithms such as PID and ADRC.
- Add sensor fusion and stable orientation estimation.
- Add communication interfaces such as CAN, UART, USB, telemetry, or an external-controller interface.
- Introduce complete platform abstractions and migrate the firmware to STM32.

The current module separation, IMU data type, and `IIMUDriver` interface are useful foundations for that work, but STM32 migration will require changes to timing, serial output, I2C access, the sensor dependency, and the build configuration.
