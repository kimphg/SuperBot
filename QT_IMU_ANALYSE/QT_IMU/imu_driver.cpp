#include "imu_driver.h"


bool IMU_driver::getIsConnected() const
{
    return isConnected;
}

IMUData IMU_driver::getMeasurement() const
{
    return measurement;
}
