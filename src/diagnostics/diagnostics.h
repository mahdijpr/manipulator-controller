#pragma once

#include "common/types.h"

class Diagnostics
{
public:
    void begin();
    void printIMU(const IMUData& data);

private:
    bool headerPrinted_ = false;
};
