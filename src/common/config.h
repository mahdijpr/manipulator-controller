#pragma once

#include <stdint.h>

// Set to 0 (or pass -DIMU_DIAGNOSTICS_ENABLED=0 in build_flags) to compile
// diagnostics out of the application update path.
#ifndef IMU_DIAGNOSTICS_ENABLED
#define IMU_DIAGNOSTICS_ENABLED 1
#endif

constexpr uint32_t SERIAL_BAUDRATE = 921600;
constexpr uint32_t IMU_I2C_CLOCK = 400000;

// Application-level IMU acquisition and startup calibration configuration.
constexpr uint32_t IMU_SAMPLE_PERIOD_US = 20000U;
constexpr uint16_t IMU_CALIBRATION_WARMUP_SAMPLES = 50U;
constexpr uint16_t IMU_CALIBRATION_SAMPLE_COUNT = 500U;

static_assert(IMU_SAMPLE_PERIOD_US > 0U, "IMU sample period must be non-zero");
static_assert(IMU_CALIBRATION_SAMPLE_COUNT > 0U, "IMU calibration requires at least one sample");
// I2C
constexpr uint8_t IMU_SDA_PIN = 12;
constexpr uint8_t IMU_SCL_PIN = 13;

// ICM42688 I2C Address
constexpr uint8_t IMU_ADDRESS = 0x68;
