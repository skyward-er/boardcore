/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/MPU9250/MPU9250.h>
#include <utils/Debug.h>

using namespace miosix;

using namespace Boardcore;

GpioPin mpuCs = GpioPin(GPIOG_BASE, 3);
SPIBus spi1(SPI1);

MPU9250* mpu = nullptr;
BMX160* bmx  = nullptr;

void initBoard()
{
    // Chip select pin as output starting high
    mpuCs.mode(miosix::Mode::OUTPUT);
    mpuCs.high();

    mpu = new MPU9250(spi1, mpuCs);

    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_8;

    BMX160Config bmxConfig;
    bmxConfig.fifoMode              = BMX160Config::FifoMode::HEADER;
    bmxConfig.fifoWatermark         = 80;
    bmxConfig.fifoInterrupt         = BMX160Config::FifoInterruptPin::PIN_INT1;
    bmxConfig.temperatureDivider    = 1;
    bmxConfig.accelerometerRange    = BMX160Config::AccelerometerRange::G_16;
    bmxConfig.gyroscopeRange        = BMX160Config::GyroscopeRange::DEG_1000;
    bmxConfig.accelerometerDataRate = BMX160Config::OutputDataRate::HZ_100;
    bmxConfig.gyroscopeDataRate     = BMX160Config::OutputDataRate::HZ_100;
    bmxConfig.magnetometerRate      = BMX160Config::OutputDataRate::HZ_100;
    bmxConfig.gyroscopeUnit         = BMX160Config::GyroscopeMeasureUnit::RAD;
    bmx = new BMX160(spi1, miosix::sensors::bmx160::cs::getPin(), bmxConfig,
                     spiConfig);

    mpu->init();
    bmx->init();
}

int main()
{
    initBoard();

    auto lastTick = getTick();
    while (true)
    {
        mpu->sample();
        bmx->sample();
        auto mpuData = mpu->getLastSample();
        auto bmxData = bmx->getLastSample();

        printf("%f,%f,%f,", mpuData.magneticFieldX, mpuData.magneticFieldY,
               mpuData.magneticFieldZ);
        printf("%f,%f,%f\n", bmxData.magneticFieldX, bmxData.magneticFieldY,
               bmxData.magneticFieldZ);

        Thread::sleepUntil(lastTick + 20);
        lastTick = getTick();
    }
}
