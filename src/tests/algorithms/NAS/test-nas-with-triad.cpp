/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <algorithms/NAS/NAS.h>
#include <algorithms/NAS/StateInitializer.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/MPU9250/MPU9250.h>
#include <sensors/SensorManager.h>
#include <sensors/calibration/SensorDataExtra.h>
#include <sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <iostream>

using namespace miosix;
using namespace Boardcore;
using namespace Eigen;

NASConfig getEKConfig();
void bmxInit();
void bmxCallback();

constexpr uint64_t CALIBRATION_TIMEOUT = 5 * 1e6;

Vector3f nedMag = Vector3f(0.47472049, 0.02757190, 0.87970463);

StateInitializer* stateInitializer;
NAS* kalman;

SPIBus spi1(SPI1);
MPU9250* mpu = nullptr;

Boardcore::SensorManager::SensorMap_t sensorsMap;
Boardcore::SensorManager* sensorManager = nullptr;

int main()
{
    bmxInit();

    sensorManager    = new SensorManager(sensorsMap);
    stateInitializer = new StateInitializer();
    kalman           = new NAS(getEKConfig());

    // Logger::getInstance().start();
    TimestampTimer::getInstance().resetTimestamp();
    sensorManager->start();

    while (true)
        Thread::sleep(1000);
}

NASConfig getEKConfig()
{
    NASConfig config;

    config.T              = 0.02f;
    config.SIGMA_BETA     = 0.0001f;
    config.SIGMA_W        = 0.3f;
    config.SIGMA_MAG      = 0.1f;
    config.SIGMA_GPS      = 10.0f;
    config.SIGMA_BAR      = 4.3f;
    config.SIGMA_POS      = 10.0;
    config.SIGMA_VEL      = 10.0;
    config.P_POS          = 1.0f;
    config.P_POS_VERTICAL = 10.0f;
    config.P_VEL          = 1.0f;
    config.P_VEL_VERTICAL = 10.0f;
    config.P_ATT          = 0.01f;
    config.P_BIAS         = 0.01f;
    config.SATS_NUM       = 6.0f;

    // Normalized magnetic field in Milan
    config.NED_MAG = nedMag;

    return config;
}

void bmxInit()
{
    mpu = new MPU9250(spi1, miosix::sensors::mpu9250::cs::getPin());

    SensorInfo info("MPU9250", 20, &bmxCallback);

    sensorsMap.emplace(std::make_pair(mpu, info));
}

void bmxCallback()
{
    static int meanCount    = 0;
    static bool calibrating = true;
    static Vector3f accMean = Vector3f::Zero();
    static Vector3f magMean = Vector3f::Zero();

    auto data = mpu->getLastSample();

    Vector3f acceleration(data.accelerationX, data.accelerationY,
                          data.accelerationZ);

    Vector3f angularVelocity(data.angularVelocityX, data.angularVelocityY,
                             data.angularVelocityZ);
    angularVelocity = angularVelocity * Constants::DEGREES_TO_RADIANS;

    SoftAndHardIronCorrector corrector({-5.15221, -56.5428, 38.2193},
                                       {0.962913, 0.987191, 1.05199});
    Vector3f magneticField;
    magneticField << corrector.correct(data);

    if (calibrating)
    {
        if (TimestampTimer::getInstance().getTimestamp() < CALIBRATION_TIMEOUT)
        {
            accMean = (accMean * meanCount + acceleration) / (meanCount + 1);
            magMean = (magMean * meanCount + acceleration) / (meanCount + 1);
            meanCount++;
        }
        else
        {
            // Now the calibration has ended, compute and log the kalman state
            calibrating = false;

            // Compute the initial kalman state
            stateInitializer->triad(accMean, magMean, nedMag);
            kalman->setX(stateInitializer->getInitX());

            // Save the state and the IMU data
            // Logger::getInstance().log(kalman->getState());
            data.accelerationX  = accMean[0];
            data.accelerationY  = accMean[1];
            data.accelerationZ  = accMean[2];
            data.magneticFieldX = magMean[0];
            data.magneticFieldY = magMean[1];
            data.magneticFieldZ = magMean[2];
            // Logger::getInstance().log(data);

            // Restart the logger to change log filename
            // Logger::getInstance().stop();
            // Logger::getInstance().start();
        }
    }
    else
    {
        // Predict step
        {
            // kalman->predictAcc(acceleration);
            kalman->predictGyro(angularVelocity);

            data.angularVelocityX = angularVelocity[0];
            data.angularVelocityY = angularVelocity[1];
            data.angularVelocityZ = angularVelocity[2];
        }

        // Correct step
        {
            magneticField.normalize();
            kalman->correctMag(magneticField);

            data.magneticFieldX = magneticField[0];
            data.magneticFieldY = magneticField[1];
            data.magneticFieldZ = magneticField[2];
        }

        auto kalmanState = kalman->getState();

        kalmanState.timestamp = TimestampTimer::getInstance().getTimestamp();
        data.accelerationTimestamp    = kalmanState.timestamp;
        data.magneticFieldTimestamp   = kalmanState.timestamp;
        data.angularVelocityTimestamp = kalmanState.timestamp;

        // Logger::getInstance().log(kalmanState);
        // Logger::getInstance().log(data);
        printf("w%fwa%fab%fbc%fc\n", kalmanState.qw, kalmanState.qx,
               kalmanState.qy, kalmanState.qz);
    }
}
