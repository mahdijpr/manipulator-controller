#pragma once

#include "sensors/imu_manager.h"
#include "diagnostics/diagnostics.h"

#include <stdint.h>

class Application
{
public:

    bool begin();

    void update();

private:

    IMUManager imu_;
    Diagnostics diagnostics_;
    uint64_t nextSampleDeadlineUs_ = 0;
};
