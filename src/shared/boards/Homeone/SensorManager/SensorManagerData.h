/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef SRC_SHARED_BOARDS_HOMEONE_SENSORMANAGER_SENSORMANAGERDATA_H
#define SRC_SHARED_BOARDS_HOMEONE_SENSORMANAGER_SENSORMANAGERDATA_H

#include <cstdint>

namespace HomeoneBoard
{


enum class SensorManagerState : uint8_t
{
    IDLE,
    LOGGING
};

//Todo: Use bitmap?
enum Sensor : uint8_t
{
    SENSOR_NONE             = 0x0000,
    SENSOR_MPU9255          = 0x0001,
    SENSOR_MAX21105         = 0x0002,
    SENSOR_ADIS             = 0x0004,
    SENSOR_AD7994           = 0x0008,
    SENSOR_PRESSURE_DIGITAL = 0x0010
};

struct SensorManagerStatus
{
    SensorManagerState state;

    // Use the or operator to signal multiple problematic sensors:
    // Ex: problematic_sensors |= SENSOR_1 | SENSOR_2;
    uint16_t problematic_sensors = 0x0000;
};

}  // namespace HomeoneBoard
#endif /* SRC_SHARED_BOARDS_HOMEONE_SENSORMANAGER_SENSORMANAGERDATA_H */
