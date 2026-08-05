# Project Directory Structure

## Current Repository Structure

```text
src/
├── main.cpp
├── app/
│   ├── application.cpp
│   └── application.h
├── calibration/
│   ├── imu_calibration.cpp
│   └── imu_calibration.h
├── common/
│   ├── config.h
│   ├── math_utils.h              # empty placeholder
│   └── types.h
├── communication/                # empty placeholder
├── control/                      # empty placeholder
├── diagnostics/
│   ├── diagnostics.cpp
│   └── diagnostics.h
├── drivers/
│   └── imu/
│       ├── imu_driver.cpp
│       ├── imu_driver.h
│       └── imu_interface.h
├── filters/
│   ├── low_pass_filter.cpp
│   └── low_pass_filter.h
├── platform/
│   ├── esp32/                    # empty placeholder
│   ├── stm32/                    # empty placeholder
│   ├── platform_time.cpp
│   └── platform_time.h
└── sensors/
    ├── imu_manager.cpp
    └── imu_manager.h
```

`src/app/` is the current application directory. There is no `src/application/` directory.

## Current Module Responsibilities

### `main.cpp`

Arduino entry point. It starts serial communication, creates the global `Application` object, calls `Application::begin()` from `setup()`, and calls `Application::update()` from `loop()`.

### `app/`

`Application` owns one `IMUManager` and one `Diagnostics` object. After successful IMU initialization, it initializes diagnostics. During each successful update it retrieves corrected IMU data and, when diagnostics are enabled, passes it once to the passive CSV observer.

### `sensors/`

`IMUManager` coordinates the concrete IMU driver and calibration module. It is the application's current access point for processed IMU data.

### `drivers/imu/`

Contains the ICM42688-specific driver and the `IIMUDriver` interface.

`ICM42688Driver` performs I2C setup, sensor initialization/configuration, raw-data acquisition, conversion through the external ICM42688 library, gyro filtering, roll/pitch calculation, and timestamping. It directly depends on Arduino `Wire` and the ICM42688 library.

`IIMUDriver` defines `begin()`, `update()`, and `getData()`. It is implemented by `ICM42688Driver`, but `IMUManager` currently owns `ICM42688Driver` directly rather than depending on the interface type.

### `calibration/`

`IMUCalibration` first accepts 50 successful warm-up samples without accumulating offsets, then averages 500 successful samples for roll, pitch, and filtered gyro values. After calibration, it subtracts these offsets from later samples. It operates on `IMUData` and has no direct sensor-library dependency.

### `filters/`

Contains `LowPassFilter`, a first-order fixed-alpha filter. Three instances belong to `ICM42688Driver` and filter gyro X/Y/Z values before calibration. The filter is not currently a standalone post-calibration pipeline stage.

### `diagnostics/`

Contains the standardized IMU CSV emitter. `Application` calls it once per successful update after `IMUManager`; it prints no row for an invalid sample. The compile-time `IMU_DIAGNOSTICS_ENABLED` macro controls this output.

### `common/`

Contains shared configuration and data types:

- `config.h`: serial, I2C, pin, and IMU-address constants.
- `types.h`: `IMUData` shared data structure.
- `math_utils.h`: empty placeholder.

### `platform/`

Currently contains only `PlatformMillis()`, which wraps Arduino `millis()`. The `esp32/` and `stm32/` directories are present but empty placeholders; they do not yet provide target-specific implementations.

### `communication/` and `control/`

Both directories are intentionally empty placeholders. No communication protocol, motor control, PID, ADRC, or manipulator-control implementation exists yet.

## Current Include and Ownership Relationships

```text
main.cpp
  -> Application
      -> IMUManager
          -> ICM42688Driver : IIMUDriver
              -> LowPassFilter x3
              -> Arduino Wire + ICM42688 library
          -> IMUCalibration
      -> Diagnostics
```

`IMUCalibration` and `Diagnostics` both use the shared `IMUData` definition. `PlatformMillis()` is used by the IMU driver for timestamps.

## Current Runtime Flow

This diagram describes the code that executes today, not the target architecture.

```text
ICM42688 sensor
  -> ICM42688Driver
      -> raw and converted measurements
      -> gyro low-pass filtering
      -> accelerometer roll/pitch calculation
      -> microsecond acquisition timestamp / dt
  -> IMUManager
  -> IMUCalibration
  -> Application
  -> corrected IMUData
  -> Diagnostics (one CSV row per valid sample when enabled)
```

## Placeholder Directories and Future Scope

`communication/`, `control/`, `platform/esp32/`, and `platform/stm32/` document intended areas of future work only. Their presence must not be interpreted as implemented communication, control, ESP32 platform adaptation, or STM32 support.
