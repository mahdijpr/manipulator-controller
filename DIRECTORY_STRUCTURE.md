# Project Directory Structure

## Verified Workspace Layout

```text
Manipulator Controller/
├── .vscode/                       # Editor configuration
├── .pio/                          # PlatformIO-generated build data
├── platformio.ini                 # ESP32-S3 / Arduino PlatformIO environment
├── analyze_imu.py                 # Offline serial-log parser, report, and plotting tool
├── PROJECT_CONTEXT.md
├── DIRECTORY_STRUCTURE.md
├── DESIGN.md
├── imu_*.csv / imu_*.txt          # Captured and cleaned validation data/reports
├── imu_analysis/                  # Generated analysis report and plots
├── include/
│   └── README                     # PlatformIO include-directory placeholder
├── lib/
│   └── README                     # PlatformIO private-library placeholder
├── test/
│   ├── host_imu_validation.cpp    # Host checks for calibration and deadline behavior
│   └── README
└── src/
    ├── main.cpp
    ├── app/
    │   ├── application.cpp
    │   ├── application.h
    │   └── periodic_deadline.h
    ├── calibration/
    │   ├── imu_calibration.cpp
    │   └── imu_calibration.h
    ├── common/
    │   ├── config.h
    │   ├── math_utils.h            # Empty placeholder
    │   └── types.h
    ├── communication/              # Empty placeholder
    ├── control/                    # Empty placeholder
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
    │   ├── esp32/                  # Empty placeholder
    │   ├── stm32/                  # Empty placeholder
    │   ├── platform_time.cpp
    │   └── platform_time.h
    └── sensors/
        ├── imu_manager.cpp
        └── imu_manager.h
```

`src/app/` is the actual application directory; there is no `src/application/` directory.

## Current Responsibilities

| Path | Current responsibility and status |
| --- | --- |
| `src/main.cpp` | Arduino entry points; starts serial and delegates to the global `Application`. |
| `src/app/` | Application scheduling and orchestration. `periodic_deadline.h` advances absolute microsecond deadlines to the first future period. |
| `src/sensors/` | `IMUManager` owns the concrete ICM42688 driver and `IMUCalibration`, and exposes final `IMUData`. |
| `src/drivers/imu/` | ICM42688-specific I2C driver plus `IIMUDriver`. The interface exists, but the manager currently owns the concrete driver. |
| `src/filters/` | First-order `LowPassFilter`; three instances filter gyro axes inside the driver before calibration. |
| `src/calibration/` | Startup warm-up and averaging offsets for roll, pitch, and filtered gyro axes. |
| `src/diagnostics/` | Compile-time-controlled CSV diagnostics emitted after a successful manager update. |
| `src/common/` | Shared configuration constants and `IMUData`; `math_utils.h` is empty. |
| `src/platform/` | Arduino/ESP32 timing wrappers. `esp32/` and `stm32/` are empty and contain no platform implementations. |
| `src/control/` | Empty placeholder; no control, PID/PD, motor, actuator, or joint implementation. |
| `src/communication/` | Empty placeholder; no transport or protocol implementation. |
| `test/` | Host-side checks for deadline progression and calibration counter behavior. |
| `analyze_imu.py` | Reads captured diagnostic text/CSV, reports timing and IMU statistics, and optionally generates plots. |

## Active Ownership Path

```text
Application
  ├── IMUManager
  │   ├── ICM42688Driver : IIMUDriver
  │   │   └── LowPassFilter x3
  │   └── IMUCalibration
  └── Diagnostics (called only when compile-time enabled)
```

The folder names reserve future areas but do not imply that control, communication, ESP32 abstraction, STM32 support, or sensor fusion is implemented.
