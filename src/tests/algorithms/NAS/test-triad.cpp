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
#include <sensors/BMX160/BMX160.h>
#include <sensors/SensorManager.h>
#include <sensors/calibration/SensorDataExtra/SensorDataExtra.h>
#include <sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.h>
#include <utils/KernelTime.h>

#include <cmath>
#include <iostream>

using namespace miosix;
using namespace Boardcore;
using namespace Eigen;

void imuInit();

Vector3f nedMag = Vector3f(0.4747, 0.0276, 0.8797);

SPIBus spi1(SPI4);
BMX160* bmx = nullptr;

int main()
{
    imuInit();
    bmx->init();

    auto lastTick = Kernel::getOldTick();
    while (true)
    {
        bmx->sample();
        auto data = bmx->getLastSample();

        Vector3f acceleration(data.accelerationX, data.accelerationY,
                              data.accelerationZ);
        Vector3f magneticField(data.magneticFieldX, data.magneticFieldY,
                               data.magneticFieldZ);

        Matrix3f A{{0.73726, 0, 0}, {0, 0.59599, 0}, {0, 0, 2.27584}};
        Vector3f b{39.22325, -17.47903, -13.81505};
        magneticField = (magneticField - b).transpose() * A;

        acceleration.normalize();
        magneticField.normalize();

        StateInitializer state;
        state.triad(acceleration, magneticField, nedMag);

        auto kalmanState = state.getInitX();
        if (!std::isnan(kalmanState(9)))
            printf("w%fwa%fab%fbc%fc\n", kalmanState(9), kalmanState(6),
                   kalmanState(7), kalmanState(8));

        Kernel::Thread::sleepUntil(lastTick + 20);
        lastTick = Kernel::getOldTick();
    }
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

    bmx = new BMX160(spi1, miosix::sensors::bmx160::cs::getPin(), bmx_config,
                     spiConfig);
}
