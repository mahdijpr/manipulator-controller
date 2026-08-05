# Firmware Design Document

This document separates the firmware that exists today from the architecture intended for future development.

# Part A — Current Implementation

## Scope

The current firmware is an ESP32-S3 based IMU acquisition and processing prototype for a future manipulator controller. It is not a motion-control firmware yet.

The current board target is `esp32-s3-devkitc-1` using PlatformIO and the Arduino framework. The firmware uses one ICM42688 over I2C at address `0x68`, with SDA on GPIO 12 and SCL on GPIO 13 at 400 kHz.

## Current Initialization Sequence

```text
setup()
  -> Serial.begin(921600)
  -> Application::begin()
      -> IMUManager::begin()
          -> ICM42688Driver::begin()
              -> Wire.begin(SDA, SCL)
              -> Wire.setClock(400 kHz)
              -> ICM42688 library initialization
              -> configure accelerometer and gyroscope
          -> IMUCalibration::begin()
      -> Diagnostics::begin() (when enabled; prints the CSV header)
```

If IMU initialization fails, `Application::begin()` prints an error and stays in an infinite delay loop.

## Current Sensor Configuration

| Channel | Full-scale range | Configured ODR |
| --- | --- | --- |
| Accelerometer | +/-4 g | 100 Hz |
| Gyroscope | +/-250 dps | 100 Hz |

The sensor configuration calls are made after library initialization. The code does not check the return values of the post-initialization configuration calls.

## Current Ownership and Interfaces

```text
Application
  owns IMUManager
  owns Diagnostics

IMUManager
  owns ICM42688Driver
  owns IMUCalibration

ICM42688Driver
  implements IIMUDriver
  owns three LowPassFilter instances
```

`IIMUDriver` defines `begin()`, `update()`, and `getData()`. Although `ICM42688Driver` implements it, `IMUManager` uses the concrete `ICM42688Driver` type directly. The interface is therefore defined but not used for runtime driver replacement.

## Current Runtime Flow

```text
loop()
  -> Application::update()
      -> compare PlatformMicros() with the next absolute 20,000 us deadline
      -> IMUManager::update() once when due
          -> ICM42688Driver::update()
              -> ICM42688::getAGT()
              -> populate raw accelerometer/gyro fields
              -> populate converted acceleration (g)
              -> read converted gyro (dps)
              -> low-pass filter gyro X/Y/Z
              -> calculate roll/pitch from accelerometer
              -> capture microsecond timestamp and dt
          -> IMUCalibration::update(driver data)
              -> process startup warm-up, average calibration samples, or subtract offsets
          -> copy corrected data to IMUManager
      -> Diagnostics emits one CSV row for a successful update (when enabled)
      -> advance the absolute deadline by whole periods until it is in the future
```

The active data flow is therefore:

```text
ICM42688
  -> ICM42688Driver
      -> gyro low-pass filters
      -> roll/pitch calculation
  -> IMUManager
  -> IMUCalibration
  -> Application
  -> no active control, communication, or logging consumer
```

Filtering is inside the current concrete driver and occurs before calibration. This differs from a separate filter layer after calibration.

## Current Data and Processing

`IMUData` contains raw accelerometer/gyro counts, converted acceleration, filtered gyro values, roll, pitch, microsecond timestamp, `dt`, calibration state, and validity.

- Accelerometer values are supplied by the library in g.
- Gyroscope values are supplied by the library in dps and then low-pass filtered with fixed alpha `0.1`.
- Roll and pitch are derived only from accelerometer values.
- `dt` is calculated from timestamps of consecutive successful samples and is not yet used by a control or fusion algorithm.
- `valid` is true only for a successfully acquired and processed sample; failed reads are not emitted as CSV data.

## Current Calibration

Calibration begins automatically after the IMU driver initializes. The first 50 successful processed samples are a warm-up period (about one second at 50 Hz): filtering and diagnostics continue normally, but these samples do not affect calibration. The next 500 successful processed samples are averaged for:

- roll,
- pitch,
- filtered gyro X,
- filtered gyro Y, and
- filtered gyro Z.

The 50 warm-up samples and 500 calibration samples are zero-based counters. Therefore, assuming the log begins at initialization and every acquisition succeeds, valid CSV row 550 (one-based) is the first row with `calibrated=1`; it is calibrated with the offsets just computed. Failed reads do not advance either counter and are not emitted. Later samples have those offsets subtracted. The code assumes the IMU is stationary during this period, but does not check for stillness, expose a recalibration command, persist offsets, or validate calibration quality.

The configuration values are `IMU_SAMPLE_PERIOD_US` (20,000), `IMU_CALIBRATION_WARMUP_SAMPLES` (50), and `IMU_CALIBRATION_SAMPLE_COUNT` (500) in `common/config.h`.

## Current Diagnostics

`Diagnostics` is a passive observer invoked only from `Application::update()` after `IMUManager` returns final calibrated data. With `IMU_DIAGNOSTICS_ENABLED` set to `1`, it prints the CSV header once after successful IMU initialization and one 15-column row per valid sample. Set the macro to `0` (or supply `-DIMU_DIAGNOSTICS_ENABLED=0`) to compile diagnostics out of the update path. Calibration emits no serial output, so it cannot corrupt the CSV stream.

`analyze_imu.py` keeps this 15-column contract. Its `--warmup-samples` and `--calibration-samples` options default to the current firmware configuration and report the observed first calibrated valid row; provide matching options if the firmware configuration is changed.

## Current Platform Abstraction

`PlatformMicros()` returns the ESP32 64-bit monotonic `esp_timer_get_time()` value and is used for scheduling and timestamps. `Application` owns scheduling: it yields full millisecond portions before a deadline, performs no fixed post-processing delay, and advances from the prior deadline rather than `now + period`. If late, it performs only the current update and advances by whole periods until the next deadline is future; it does not perform catch-up reads. The firmware still directly uses Arduino `Serial`, delay, and `Wire`, as well as the Arduino ICM42688 library. Consequently, the current implementation is not yet platform-independent.

## Current Limitations

- No motor, servo, encoder, kinematics, control, or communication implementation.
- No yaw estimate or sensor-fusion algorithm.
- No acceleration calibration or filtering.
- No runtime sensor health state, read-error counter, or recovery behavior.
- Physical-board capture is still required to confirm the achieved 50 Hz timing and jitter.
- The global sensor object and timestamp state in the driver make the current driver tightly coupled to one IMU instance.

# Part B — Future Architecture

Everything in this section is a future plan. It does not describe current firmware behavior.

## Intended Direction

The project is intended to grow into a manipulator controller while preserving a clear separation between application logic, processing/control algorithms, device drivers, and target-specific platform services.

## Future Hardware Abstraction and Driver Replacement

The existing `IIMUDriver` interface can become the driver substitution boundary when `IMUManager` receives or owns an `IIMUDriver` abstraction rather than a concrete `ICM42688Driver`. This would support replacement with another IMU driver, such as a BMI270 implementation, without making the sensor manager depend on a particular sensor class.

Future platform abstractions should cover timing, delay/scheduling, I2C, logging/serial output, and target configuration. This is needed before describing STM32 migration as supported.

## Future STM32 Migration

STM32 migration is planned work. It will require:

- STM32 target/build configuration.
- STM32 implementations for timing, I2C, serial/logging, and hardware configuration.
- An ICM42688 driver/library compatible with the selected STM32 environment.
- Removal or encapsulation of Arduino-specific dependencies outside platform code.

The current empty `platform/stm32/` directory is only a placeholder.

## Future Processing, Communication, and Control

Potential future modules include:

- Sensor fusion for stable orientation estimation.
- Communication interfaces such as UART, CAN, USB, telemetry, or an external-controller interface.
- Manipulator hardware support: actuators, encoders, joints, and safety handling.
- Motion-control algorithms including PID, ADRC, and trajectory generation.

A future target flow may resemble:

```text
Sensors -> driver interfaces -> processing / sensor fusion
        -> control algorithms -> actuator interfaces
        -> manipulator hardware

Application / communication coordinate configuration and commands.
Platform services supply target-specific timing, I2C, logging, and transport support.
```

This is deliberately different from the current flow: the present firmware contains only one concrete IMU path and has no active control, actuator, communication, or fusion stage.
