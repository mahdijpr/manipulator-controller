#include "imu_manager.h"

bool IMUManager::begin()
{
    if(!driver_.begin())
        return false;


    calibration_.begin();


    return true;
}

bool IMUManager::update()
{

    if(!driver_.update())
        return false;


    calibration_.update(
        driver_.getData()
    );


    data_ =
        calibration_.getData();


    return true;
}

const IMUData& IMUManager::getData() const
{
    return data_;
}