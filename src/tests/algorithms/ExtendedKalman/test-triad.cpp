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

#include <algorithms/ExtendedKalman/StateInitializer.h>
#include <miosix.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/LIS3MDL/LIS3MDL.h>
#include <sensors/SensorManager.h>

#include <cmath>
#include <iostream>

using namespace miosix;
using namespace Boardcore;
using namespace Eigen;

void bmxInit();
void bmxCallback();

Vector3f nedMag = Vector3f(0.4747, 0.0276, 0.8797);

SPIBus spi1(SPI1);
BMX160* bmx160 = nullptr;

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

    Vector3f acceleration(data.accelerationX, data.accelerationY,
                          data.accelerationZ);
    Vector3f magneticField(data.magneticFieldX, data.magneticFieldY,
                           data.magneticFieldZ);

    // Apply correction
    Vector3f acc_b(0.3763, -0.1445, -0.1010);
    acceleration -= acc_b;
    Vector3f mag_b{21.0730033648425, -24.3997259703105, -2.32621524742862};
    Matrix3f A{{0.659926504672263, 0, 0},
               {0, 0.662442130094073, 0},
               {0, 0, 2.28747567094359}};
    magneticField = (magneticField - mag_b).transpose() * A;

    acceleration.normalize();
    magneticField.normalize();

    StateInitializer state;
    state.triad(acceleration, magneticField, nedMag);

    auto kalmanState = state.getInitX();
    if (!std::isnan(kalmanState(9)))
        printf("w%fwa%fab%fbc%fc\n", kalmanState(9), kalmanState(6),
               kalmanState(7), kalmanState(8));
}
