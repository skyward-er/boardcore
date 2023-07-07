/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Federico Lolli
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

#include <logger/Logger.h>
#include <utils/Debug.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;

Logger& logger = Logger::getInstance();

using GpioSck  = Gpio<GPIOB_BASE, 3>;
using GpioMiso = Gpio<GPIOB_BASE, 4>;
using GpioMosi = Gpio<GPIOB_BASE, 5>;
using GpioCS   = Gpio<GPIOA_BASE, 15>;

int main()
{
    // start logger
    TRACE("Setting up Logger...\n");
    if (!logger.start())
    {
        TRACE("Logger failed to start, aborting.\n");
        return 1;
    }

    TRACE("Setting up Pins...\n");
    GpioCS::mode(Mode::OUTPUT);
    GpioCS::high();

    GpioSck::mode(Mode::ALTERNATE);
    GpioSck::alternateFunction(5);

    GpioMiso::mode(Mode::ALTERNATE);
    GpioMiso::alternateFunction(5);

    GpioMosi::mode(Mode::ALTERNATE);
    GpioMosi::alternateFunction(5);

    TRACE("Setting up SPI...\n");
    SPIBus bus(SPI3);

    SPIBusConfig busConfig;
    busConfig.clockDivider = SPI::ClockDivider::DIV_32;

    LIS2MDL::Config config;
    config.odr                = LIS2MDL::ODR_100_HZ;
    config.deviceMode         = LIS2MDL::MD_CONTINUOUS;
    config.temperatureDivider = 5;

    TRACE("Setting up Sensor...\n");
    LIS2MDL sensor(bus, GpioCS::getPin(), busConfig, config);

    if (!sensor.init())
    {
        TRACE("LIS3MDL: Init failed\n");
        return 1;
    }

    TRACE("Doing self test!\n");
    bool ok = sensor.selfTest();
    if (!ok)
    {
        TRACE("Error: selfTest() returned false!\n");
    }

    TRACE("Now printing some sensor data:\n");
    Thread::sleep(100);

    for (int i = 0; i < 10; i++)
    {
        sensor.sample();
        LIS2MDLData data __attribute__((unused)) = sensor.getLastSample();
        TRACE("%f C | x: %f | y: %f | z %f\n", data.temperature,
              data.magneticFieldX, data.magneticFieldY, data.magneticFieldZ);
        miosix::Thread::sleep(2000);
    }

    TRACE("Completed\n");
    logger.stop();
    return 0;
}
