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
#include <logger/Logger.h>
#include <miosix.h>
#include <sensors/MPU9250/MPU9250.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSerial.h>
#include <utils/AeroUtils/AeroUtils.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <iostream>

using namespace miosix;
using namespace Boardcore;
using namespace Eigen;

NASConfig getEKConfig();
void setInitialOrientation();
void init();
void step();
void print();

Vector3f nedMag   = Vector3f(0.4747, 0.0276, 0.8797);
Vector2f startPos = Vector2f(45.501141, 9.156281);

NAS* nas;

SPIBus spi1(SPI1);
MPU9250* imu      = nullptr;
UBXGPSSerial* gps = nullptr;

int main()
{
    init();

    nas = new NAS(getEKConfig());
    nas->setReferenceValues({0, 0, 0, 110000, 20 + 273.5});
    setInitialOrientation();

    TaskScheduler scheduler;
    scheduler.addTask(step, 20, TaskScheduler::Policy::RECOVER);
    scheduler.addTask(print, 250);
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
    Matrix<float, 13, 1> x = Matrix<float, 13, 1>::Zero();

    // Set quaternions
    Vector4f q = SkyQuaternion::eul2quat({0, 0, 0});
    x(6)       = q(0);
    x(7)       = q(1);
    x(8)       = q(2);
    x(9)       = q(3);

    nas->setX(x);
}

void init()
{
    imu = new MPU9250(spi1, sensors::mpu9250::cs::getPin());
    imu->init();

    gps = new UBXGPSSerial(USARTInterface::Baudrate::B38400, 10, USART2,
                           USARTInterface::Baudrate::B38400);
    gps->init();
    gps->start();

    Logger::getInstance().start();
}

void step()
{
    imu->sample();
    gps->sample();
    auto imuData = imu->getLastSample();
    auto gpsData = gps->getLastSample();

    Vector3f acceleration(imuData.accelerationX, imuData.accelerationY,
                          imuData.accelerationZ);
    Vector3f angularVelocity(imuData.angularVelocityX, imuData.angularVelocityY,
                             imuData.angularVelocityZ);
    Vector3f magneticField(imuData.magneticFieldX, imuData.magneticFieldY,
                           imuData.magneticFieldZ);

    Vector2f gpsPos(gpsData.latitude, gpsData.longitude);
    gpsPos = Aeroutils::geodetic2NED(gpsPos, startPos);
    Vector2f gpsVel(gpsData.velocityNorth, gpsData.velocityNorth);
    Vector4f gpsCorrection;
    // cppcheck-suppress constStatement
    gpsCorrection << gpsPos, gpsVel;

    // Calibration
    {
        Vector3f biasAcc(-0.1255, 0.2053, -0.2073);
        acceleration -= biasAcc;
        Vector3f bias(-0.0291, 0.0149, 0.0202);
        angularVelocity -= bias;
        Vector3f offset(15.9850903462129, -15.6775071377074, -33.8438469147423);
        magneticField -= offset;
        magneticField = {magneticField[1], magneticField[0], -magneticField[2]};
    }

    // Predict step
    nas->predictGyro(angularVelocity);
    if (gpsPos[0] < 1e3 && gpsPos[0] > -1e3 && gpsPos[1] < 1e3 &&
        gpsPos[1] > -1e3)
        nas->predictAcc(acceleration);

    // Correct step
    magneticField.normalize();
    nas->correctMag(magneticField);
    acceleration.normalize();
    nas->correctAcc(acceleration);
    if (gpsData.fix)
        nas->correctGPS(gpsCorrection);
    nas->correctBaro(100000);

    auto nasState = nas->getState();
    Logger::getInstance().log(imuData);
    Logger::getInstance().log(gpsData);
    Logger::getInstance().log(nasState);
}

void print()
{
    auto gpsData  = gps->getLastSample();
    auto nasState = nas->getState();

    Vector2f gpsPos(gpsData.latitude, gpsData.longitude);
    gpsPos = Aeroutils::geodetic2NED(gpsPos, startPos);

    printf("%d, %f, %f, %f, %f, %f, %f\n", gpsData.fix, gpsPos[0], gpsPos[1],
           nasState.n, nasState.e, nasState.vn, nasState.ve);
}
