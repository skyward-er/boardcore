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

#include <algorithms/kalman/ExtendedKalmanEigen/ExtendedKalmanEigen.h>
#include <math/SkyQuaternion.h>
#include <miosix.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/SensorManager.h>

#include <iostream>

using namespace miosix;
using namespace Boardcore;
using namespace Eigen;

ExtendedKalmanConfig getEKConfig();
void setInitialOrientation();
void bmxInit();
void bmxCallback();

ExtendedKalmanEigen* kalman;
SkyQuaternion quat;

SPIBus spi1(SPI1);
BMX160* bmx160                          = nullptr;
Boardcore::SensorManager* sensorManager = nullptr;

int main()
{
    bmxInit();

    Logger::getInstance().start();

    kalman = new ExtendedKalmanEigen(getEKConfig());
    setInitialOrientation();

    sensorManager->start();

    // Thread::sleep(30 * 1000);

    // sensorManager->stop();
    // Logger::getInstance().stop();

    while (true)
        Thread::sleep(1000);
}

ExtendedKalmanConfig getEKConfig()
{
    ExtendedKalmanConfig config;

    config.T              = 0.02f;
    config.SIGMA_BETA     = 0.0001f;
    config.SIGMA_W        = 0.3f;
    config.SIGMA_MAG      = 0.05f;
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
    config.NED_MAG = Vector3f(0.4742f, 0.025f, 0.8801f);

    return config;
}

void setInitialOrientation()
{
    Eigen::Matrix<float, 13, 1> x;

    // Set quaternions
    Eigen::Vector4f q = quat.eul2quat({0, 0, 0});
    x(6)              = q(0);
    x(7)              = q(1);
    x(8)              = q(2);
    x(9)              = q(3);

    kalman->setX(x);
}

void bmxInit()
{

    SPIBusConfig spi_cfg;
    spi_cfg.clockDivider = SPI::ClockDivider::DIV_8;

    BMX160Config bmx_config;
    bmx_config.fifoMode      = BMX160Config::FifoMode::HEADER;
    bmx_config.fifoWatermark = 80;
    bmx_config.fifoInterrupt = BMX160Config::FifoInterruptPin::PIN_INT1;

    bmx_config.temperatureDivider = 1;

    bmx_config.accelerometerRange = BMX160Config::AccelerometerRange::G_16;

    bmx_config.gyroscopeRange = BMX160Config::GyroscopeRange::DEG_1000;

    bmx_config.accelerometerDataRate = BMX160Config::OutputDataRate::HZ_1600;
    bmx_config.gyroscopeDataRate     = BMX160Config::OutputDataRate::HZ_1600;
    bmx_config.magnetometerRate      = BMX160Config::OutputDataRate::HZ_100;

    bmx_config.gyroscopeUnit = BMX160Config::GyroscopeMeasureUnit::RAD;

    bmx160 = new BMX160(spi1, miosix::sensors::bmx160::cs::getPin(), bmx_config,
                        spi_cfg);

    SensorInfo info("BMX160", 20, &bmxCallback);

    Boardcore::SensorManager::SensorMap_t sensorsMap;
    sensorsMap.emplace(std::make_pair(bmx160, info));

    sensorManager = new SensorManager(sensorsMap);
}

void bmxCallback()
{
    auto data = bmx160->getLastSample();

    Vector3f acceleration(data.accelerationX, data.accelerationY,
                          data.accelerationZ);
    kalman->predict(acceleration);

    Vector3f angularVelocity(data.angularVelocityX, data.angularVelocityY,
                             data.angularVelocityZ);
    kalman->predictMEKF(angularVelocity);

    Vector3f magneticField(data.magneticFieldX, data.magneticFieldY,
                           data.magneticFieldZ);
    magneticField.normalize();
    kalman->correctMEKF(magneticField);

    auto kalmanState    = kalman->getState();
    auto kalmanRotation = quat.quat2eul(Vector4f(
        kalmanState(6), kalmanState(7), kalmanState(8), kalmanState(9)));

    data.accelerationTimestamp = TimestampTimer::getInstance().getTimestamp();

    printf("Orientation: %6.4f, %6.2f, %6.4f\n", kalmanRotation(0),
           kalmanRotation(1), kalmanRotation(2));

    Logger::getInstance().log(data);
}