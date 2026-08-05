#pragma once

#include "common/types.h"

class IIMUDriver
{
public:

    virtual ~IIMUDriver() = default;

    virtual bool begin() = 0;

    virtual bool update() = 0;

    virtual const IMURawMeasurements& getData() const = 0;
};
