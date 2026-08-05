#include "imu_driver.h"

#include <math.h>
#include <Wire.h>
#include <ICM42688.h>

#include "common/config.h"
#include "platform/platform_time.h"


ICM42688 imu(
    Wire,
    IMU_ADDRESS,
    IMU_SDA_PIN,
    IMU_SCL_PIN
);


// Constructor
ICM42688Driver::ICM42688Driver()
:
gyroXFilter(0.1f),
gyroYFilter(0.1f),
gyroZFilter(0.1f)
{

}


bool ICM42688Driver::begin()
{
    data_ = IMUData{};
    lastTimestampUs_ = 0;

    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);

    Wire.setClock(IMU_I2C_CLOCK);


    int status = imu.begin();

    if(status < 0)
        return false;


    imu.setGyroFS(ICM42688::dps250);
    imu.setAccelFS(ICM42688::gpm4);


    imu.setGyroODR(ICM42688::odr100);
    imu.setAccelODR(ICM42688::odr100);


    return true;
}



bool ICM42688Driver::update()
{
    data_.valid = false;

    if (imu.getAGT() != 1)
        return false;

    // getAGT() has acquired the complete sensor sample at this point.
    const uint64_t timestampUs = PlatformMicros();

    // Raw, unfiltered and uncalibrated ADC counts.
    data_.rawAccelX = imu.rawAccX();
    data_.rawAccelY = imu.rawAccY();
    data_.rawAccelZ = imu.rawAccZ();

    data_.rawGyroX = imu.rawGyrX();
    data_.rawGyroY = imu.rawGyrY();
    data_.rawGyroZ = imu.rawGyrZ();



    // Accelerometer
    data_.accelXG = imu.accX();
    data_.accelYG = imu.accY();
    data_.accelZG = imu.accZ();



    // Gyro raw DPS
    float gx = imu.gyrX();
    float gy = imu.gyrY();
    float gz = imu.gyrZ();


    // Gyro filtered
    data_.gyroXDegS = gyroXFilter.update(gx);
    data_.gyroYDegS = gyroYFilter.update(gy);
    data_.gyroZDegS = gyroZFilter.update(gz);



    // Accelerometer angle
    data_.rollDeg =
        atan2(data_.accelYG, data_.accelZG)
        * 180.0f / PI;


    data_.pitchDeg =
        atan2(
            -data_.accelXG,
            sqrt(
                data_.accelYG * data_.accelYG +
                data_.accelZG * data_.accelZG
            )
        )
        * 180.0f / PI;



    data_.timestampUs = timestampUs;
    data_.dtSeconds = 0.0f;

    if (lastTimestampUs_ != 0)
    {
        data_.dtSeconds =
            static_cast<float>(timestampUs - lastTimestampUs_) / 1000000.0f;
    }

    lastTimestampUs_ = timestampUs;
    data_.valid = true;

    return true;
}



const IMUData& ICM42688Driver::getData() const
{
    return data_;
}
