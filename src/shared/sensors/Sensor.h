/* Sensors Base Classes
 *
 * Copyright (c) 2016 Skyward Experimental Rocketry
 * Authors: Alain Carlucci
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef SENSORS_H
#define SENSORS_H
#include <Common.h>
#include <drivers/spi/SensorSpi.h>
#include <math/Quaternion.h>
#include <math/Vec3.h>

/** Sensors class diagram
 *               ________
 *              | Sensor |                      <- Sensor parent
 *      _______/ -------- \________
 * ____/_______   _|__________   __\_______
 *| GyroSensor | | ...Sensor  | | AnySensor|    <- Virtual childs
 * ------------   ------------   ----------
 *    |     _______/       \       /
 *  __|____/_             __\_____/_
 * | ABC1234 |           | LOL12345 |           <- You write these
 *  ---------             ----------
 */

class Sensor
{
public:
    /** Here the code to initialize this sensor */
    // virtual bool init() = 0;

    /** Self test code
     * It should return a boolean:
     *    True  = sensor ok
     *    False = sensor ko, write into last_error the error code.
     * Anyone should be able to call getLastError() and read the error.
     */
    virtual bool selfTest() = 0;

    virtual std::vector<SPIRequest> buildDMARequest()
    {
        // printf("** SENSOR::buildDMARequest **\n");
        return std::vector<SPIRequest>();
    }

    virtual void onDMAUpdate(const SPIRequest& req)
    {
        // printf("** SENSOR::onDMAUpdate **\n");
    }

    /**
     * This method is called once every N msec, read new values and
     * store them in local variables.
     *
     * You should check getLastError() if this function returns false.
     */
    virtual bool onSimpleUpdate() = 0;

    /** Return last error code */
    uint8_t getLastError() const { return last_error; }

    /** Errors (for each subsensor) */
    // clang-format off
        enum eErrors
        {
            ERR_NOT_ME              = 0x01,
            ERR_RESET_TIMEOUT       = 0x02,
            ERR_BUS_FAULT           = 0x03, // A bus op has encountered an error
            ERR_X_SELFTEST_FAIL     = 0x04,
            ERR_Y_SELFTEST_FAIL     = 0x05,
            ERR_Z_SELFTEST_FAIL     = 0x06,
            ERR_ACCEL_SELFTEST      = 0x07,
            ERR_GYRO_SELFTEST       = 0x08,
            ERR_CANT_TALK_TO_CHILD  = 0x09, // MPU9250 can't talk to AK8963
        };
    // clang-format on

protected:
    uint8_t last_error = 0;
};

class GyroSensor : public virtual Sensor
{
public:
    const Vec3* gyroDataPtr() const { return &mLastGyro; }

protected:
    Vec3 mLastGyro;
};

class AccelSensor : public virtual Sensor
{
public:
    const Vec3* accelDataPtr() const { return &mLastAccel; }

protected:
    Vec3 mLastAccel;
};

class CompassSensor : public virtual Sensor
{
public:
    const Vec3* compassDataPtr() const { return &mLastCompass; }

protected:
    Vec3 mLastCompass;
};

class TemperatureSensor : public virtual Sensor
{
public:
    const float* tempDataPtr() const { return &mLastTemp; }

protected:
    float mLastTemp;
};

class HumiditySensor : public virtual Sensor
{
public:
    const float* humidityDataPtr() const { return &mLastHumidity; }

protected:
    float mLastHumidity;
};

class PressureSensor : public virtual Sensor
{
public:
    const float* pressureDataPtr() const { return &mLastPressure; }

protected:
    float mLastPressure;
};

class AltitudeSensor : public virtual Sensor
{
public:
    const float* altitudeDataPtr() const { return &mLastAltitude; }

protected:
    float mLastAltitude;
};

class DebugIntSensor : public virtual Sensor
{
public:
    const int* debugIntPtr() const { return &mDebugInt; }

protected:
    int mDebugInt;
};

#endif /* ifndef SENSORS_H */
