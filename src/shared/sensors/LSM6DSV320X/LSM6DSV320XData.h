#pragma once

#include <sensors/SensorData.h>

namespace Boardcore
{

struct LSM6DSV320XData
{
    AccelerometerData lowG;
    AccelerometerData highG;
    GyroscopeData gyroscope;
    TemperatureData temperature;

    LSM6DSV320XData()
        : lowG{0, 0.0, 0.0, 0.0},
          highG{0, 0.0, 0.0, 0.0},
          gyroscope{0, 0.0, 0.0, 0.0},
          temperature{0, 0.0}
    {}
};

}