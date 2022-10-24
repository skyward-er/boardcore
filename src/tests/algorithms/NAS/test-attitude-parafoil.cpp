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
#include <sensors/MPU9250/MPU9250.h>
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
MPU9250* imu = nullptr;

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
    imu = new MPU9250(spi1, sensors::mpu9250::cs::getPin());
    imu->init();
}

void imuStep()
{
    imu->sample();
    auto data = imu->getLastSample();
    Vector3f acceleration(data.accelerationX, data.accelerationY,
                          data.accelerationZ);
    Vector3f angularSpeed(data.angularSpeedX, data.angularSpeedY,
                          data.angularSpeedZ);
    Vector3f magneticField(data.magneticFieldX, data.magneticFieldY,
                           data.magneticFieldZ);

    // Calibration
    {
        Vector3f bias(0.0218462708018154, 0.0281755574886535,
                      0.0264119470499244);
        angularSpeed -= bias;
        Vector3f offset(15.9850903462129, -15.6775071377074, -33.8438469147423);
        magneticField -= offset;
        magneticField = {magneticField[1], magneticField[0], -magneticField[2]};
    }

    acceleration.normalize();
    magneticField.normalize();

    // Predict step
    nas->predictGyro(angularSpeed);

    // Correct step
    nas->correctMag(magneticField);
    nas->correctAcc(acceleration);

    auto nasState = nas->getState();
    printf("w%fwa%fab%fbc%fc\n", nasState.qw, nasState.qx, nasState.qy,
           nasState.qz);
}
