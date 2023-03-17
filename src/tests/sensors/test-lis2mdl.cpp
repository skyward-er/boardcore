/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Riccardo Musso
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
#include <drivers/usart/USART.h>
#include <miosix.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <utils/Debug.h>

using namespace Boardcore;
using namespace miosix;

int main()
{
    GpioPin cs(GPIOA_BASE, 15), miso(GPIOA_BASE, 6), mosi(GPIOA_BASE, 7),
        clk(GPIOA_BASE, 5), tx(GPIOA_BASE, 2), rx(GPIOA_BASE, 3);

    cs.mode(Mode::OUTPUT);
    cs.high();

    clk.mode(Mode::ALTERNATE);
    clk.alternateFunction(5);
    clk.speed(Speed::_100MHz);

    miso.mode(Mode::ALTERNATE);
    miso.alternateFunction(5);

    mosi.mode(Mode::ALTERNATE);
    mosi.alternateFunction(5);

    // Serial data write on Matlab
    rx.mode(Mode::ALTERNATE);
    rx.alternateFunction(7);

    tx.mode(Mode::ALTERNATE);
    tx.alternateFunction(7);

    SPIBus bus(SPI1);
    USART usart(USART2, USARTInterface::Baudrate::B115200);
    usart.init();

    SPIBusConfig busConfig;
    busConfig.clockDivider = SPI::ClockDivider::DIV_256;
    busConfig.mode         = SPI::Mode::MODE_3;

    LIS2MDL::Config config;
    config.odr                = LIS2MDL::ODR_10_HZ;
    config.deviceMode         = LIS2MDL::MD_CONTINUOUS;
    config.temperatureDivider = 5;

    LIS2MDL sensor(bus, cs, busConfig, config);

    if (!sensor.init())
    {
        TRACE("LIS2MDL: Init failed");
        return 1;
    }
    TRACE("LIS2MDL: Init done\n");

    TRACE("Doing self test!\n");
    if (!sensor.selfTest())
    {
        TRACE("Error: selfTest() returned false!\n");
    }
    TRACE("selfTest returned true\n");
    TRACE("Now printing some sensor data:\n");
    Thread::sleep(3000);
    float sensorData[3];

    while (true)
    {
        sensor.sample();
        LIS2MDLData data = sensor.getLastSample();
        TRACE("%f C | x: %f | y: %f | z: %f\n", data.temperature,
              data.magneticFieldX, data.magneticFieldY, data.magneticFieldZ);
        sensorData[0] = data.magneticFieldX;
        sensorData[1] = data.magneticFieldY;
        sensorData[2] = data.magneticFieldZ;
        usart.write(sensorData, 3 * sizeof(float));
        miosix::Thread::sleep(10);
    }
}
