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
#include <miosix.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/SensorManager.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <iostream>

using namespace miosix;
using namespace Boardcore;
using namespace Eigen;

NASConfig getEKConfig();
void setInitialOrientation();
void imuInit();
void imuStep();

Vector3f nedMag = Vector3f(0.4747, 0.0276, 0.8797);

NAS* nas;

SPIBus spi1(SPI1);
BMX160* imu = nullptr;

int main()
{
    imuInit();

    nas = new NAS(getEKConfig());
    setInitialOrientation();

    TaskScheduler scheduler;
    scheduler.addTask(imuStep, 20);
    scheduler.start();

    while (true)
        Thread::sleep(1000);
}

NASConfig getEKConfig()
{
    NASConfig config;

    config.T              = 0.02f;
    config.SIGMA_BETA     = 0.0001f;
    config.SIGMA_W        = 0.3f;
    config.SIGMA_ACC      = 0.1f;
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
    config.NED_MAG        = nedMag;

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

    nas->setX(x);
}

void imuInit()
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

    imu = new BMX160(spi1, miosix::sensors::bmx160::cs::getPin(), bmx_config,
                     spiConfig);
    imu->init();
}

void imuStep()
{
    imu->sample();
    auto data = imu->getLastSample();
    Vector3f acceleration(data.accelerationX, data.accelerationY,
                          data.accelerationZ);
    Vector3f angularVelocity(data.angularVelocityX, data.angularVelocityY,
                             data.angularVelocityZ);
    Vector3f magneticField(data.magneticFieldX, data.magneticFieldY,
                           data.magneticFieldZ);

    // Calibration
    {
        Vector3f offset{-1.63512255486542, 3.46523431469979, -3.08516033954451};
        angularVelocity = angularVelocity - offset;
        angularVelocity = angularVelocity / 180 * Constants::PI / 10;
        Vector3f b{21.5356818859811, -22.7697302909894, -2.68219304319269};
        Matrix3f A{{0.688760050772712, 0, 0},
                   {0, 0.637715211784480, 0},
                   {0, 0, 2.27669720320908}};
        magneticField = (magneticField - b).transpose() * A;
    }

    acceleration.normalize();
    magneticField.normalize();

    // Predict step
    nas->predictGyro(angularVelocity);

    // Correct step
    nas->correctMag(magneticField);
    nas->correctAcc(acceleration);

    // std::cout << acceleration.transpose() << angularVelocity.transpose()
    //           << magneticField.transpose() << std::endl;

    auto nasState = nas->getState();
    printf("w%fwa%fab%fbc%fc\n", nasState.qw, nasState.qx, nasState.qy,
           nasState.qz);
}
