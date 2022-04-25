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

#include <algorithms/NAS/StateInitializer.h>
#include <miosix.h>
#include <sensors/MPU9250/MPU9250.h>
#include <sensors/SensorManager.h>
#include <sensors/calibration/SensorDataExtra.h>
#include <sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.h>

#include <cmath>
#include <iostream>

using namespace miosix;
using namespace Boardcore;
using namespace Eigen;

void bmxInit();
void bmxCallback();

Vector3f nedMag = Vector3f(0.4747, 0.0276, 0.8797);

SPIBus spi1(SPI1);
MPU9250* mpu = nullptr;

Boardcore::SensorManager::SensorMap_t sensorsMap;
Boardcore::SensorManager* sensorManager = nullptr;

int main()
{
    bmxInit();

    sensorManager = new SensorManager(sensorsMap);
    sensorManager->start();

    while (true)
        Thread::sleep(1000);
}

void bmxInit()
{
    mpu = new MPU9250(spi1, miosix::sensors::mpu9250::cs::getPin());

    SensorInfo info("MPU9250", 20, &bmxCallback);

    sensorsMap.emplace(std::make_pair(mpu, info));
}

void bmxCallback()
{
    auto data = mpu->getLastSample();

    Vector3f acceleration(data.accelerationX, data.accelerationY,
                          data.accelerationZ);

    SoftAndHardIronCorrector corrector({-5.15221, -56.5428, 38.2193},
                                       {0.962913, 0.987191, 1.05199});
    Vector3f magneticField;
    magneticField << (MagnetometerData)data;

    magneticField << corrector.correct(data);

    acceleration.normalize();
    magneticField.normalize();

    StateInitializer state;
    state.triad(acceleration, magneticField, nedMag);

    std::cout << acceleration.transpose();
    auto kalmanState = state.getInitX();
    if (!std::isnan(kalmanState(9)))
        printf("w%fwa%fab%fbc%fc\n", kalmanState(9), kalmanState(6),
               kalmanState(7), kalmanState(8));
}
