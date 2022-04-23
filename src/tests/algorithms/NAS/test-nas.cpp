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
#include <miosix.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/LIS3MDL/LIS3MDL.h>
#include <sensors/SensorManager.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <iostream>

using namespace miosix;
using namespace Boardcore;
using namespace Eigen;

NASConfig getEKConfig();
void setInitialOrientation();
void bmxInit();
void bmxCallback();

NAS* kalman;

SPIBus spi1(SPI1);
BMX160* bmx160 = nullptr;

Boardcore::SensorManager::SensorMap_t sensorsMap;
Boardcore::SensorManager* sensorManager = nullptr;

int main()
{
    bmxInit();

    sensorManager = new SensorManager(sensorsMap);
    kalman        = new NAS(getEKConfig());
    setInitialOrientation();

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
    config.SIGMA_POS      = 10.0f;
    config.SIGMA_VEL      = 10.0f;
    config.P_POS          = 1.0f;
    config.P_POS_VERTICAL = 10.0f;
    config.P_VEL          = 1.0f;
    config.P_VEL_VERTICAL = 10.0f;
    config.P_ATT          = 0.01f;
    config.P_BIAS         = 0.01f;
    config.SATS_NUM       = 6.0f;

    // Normalized magnetic field in Milan
    config.NED_MAG = Vector3f(0.0812241, -0.963085, 0.256649);

    return config;
}

void setInitialOrientation()
{
    Eigen::Matrix<float, 13, 1> x;

    // Set quaternions
    Eigen::Vector4f q = SkyQuaternion::eul2quat({0, 0, 0});
    x(6)              = q(0);
    x(7)              = q(1);
    x(8)              = q(2);
    x(9)              = q(3);

    kalman->setX(x);
}

void bmxInit()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_8;

    BMX160Config bmx_config;
    bmx_config.fifoMode      = BMX160Config::FifoMode::HEADER;
    bmx_config.fifoWatermark = 80;
    bmx_config.fifoInterrupt = BMX160Config::FifoInterruptPin::PIN_INT1;

    bmx_config.temperatureDivider = 1;

    bmx_config.accelerometerRange = BMX160Config::AccelerometerRange::G_16;

    bmx_config.gyroscopeRange = BMX160Config::GyroscopeRange::DEG_1000;

    bmx_config.accelerometerDataRate = BMX160Config::OutputDataRate::HZ_100;
    bmx_config.gyroscopeDataRate     = BMX160Config::OutputDataRate::HZ_100;
    bmx_config.magnetometerRate      = BMX160Config::OutputDataRate::HZ_100;

    bmx_config.gyroscopeUnit = BMX160Config::GyroscopeMeasureUnit::RAD;

    bmx160 = new BMX160(spi1, miosix::sensors::bmx160::cs::getPin(), bmx_config,
                        spiConfig);

    SensorInfo info("BMX160", 20, &bmxCallback);

    sensorsMap.emplace(std::make_pair(bmx160, info));
}

void bmxCallback()
{
    auto data = bmx160->getLastSample();

    // Predict step
    {
        Vector3f angularVelocity(data.angularVelocityX, data.angularVelocityY,
                                 data.angularVelocityZ);
        Vector3f offset{-1.63512255486542, 3.46523431469979, -3.08516033954451};
        angularVelocity = angularVelocity - offset;
        angularVelocity = angularVelocity / 180 * Constants::PI / 10;

        kalman->predictGyro(angularVelocity);
    }

    // Correct with acclelerometer
    // {
    //     Vector3f angularVelocity(data.accelerationX, data.accelerationY,
    //                              data.accelerationZ);

    //     kalman->correctAcc(angularVelocity);
    // }

    // Correct step
    {
        Vector3f magneticField(data.magneticFieldX, data.magneticFieldY,
                               data.magneticFieldZ);

        // Apply correction
        Vector3f b{21.0730033648425, -24.3997259703105, -2.32621524742862};
        Matrix3f A{{0.659926504672263, 0, 0},
                   {0, 0.662442130094073, 0},
                   {0, 0, 2.28747567094359}};
        magneticField = (magneticField - b).transpose() * A;

        magneticField.normalize();
        kalman->correctMag(magneticField);
    }

    auto kalmanState = kalman->getState();

    printf("w%fwa%fab%fbc%fc\n", kalmanState.qw, kalmanState.qx, kalmanState.qy,
           kalmanState.qz);
}
