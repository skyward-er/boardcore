/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Fabrizio Monti
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
#include <sensors/Vectornav/VN100/VN100Spi.h>

using namespace miosix;
using namespace Boardcore;

int main()
{
    // spi setup
    SPIBus bus(SPI3);

    GpioPin csPin(GPIOE_BASE, 3);  // PE3 CS
    csPin.mode(Mode::OUTPUT);
    GpioPin clockPin(GPIOB_BASE, 3);  // PB3 CK
    clockPin.mode(Mode::ALTERNATE);
    clockPin.alternateFunction(6);
    GpioPin misoPin(GPIOB_BASE, 4);  // PB4 MISO
    misoPin.mode(Mode::ALTERNATE);
    misoPin.alternateFunction(6);
    GpioPin mosiPin(GPIOB_BASE, 5);  // PB5 MOSI
    mosiPin.mode(Mode::ALTERNATE);
    mosiPin.alternateFunction(6);

    GpioPin intPin(GPIOC_BASE, 15);  // PC15 interrupt pin
    intPin.mode(Mode::INPUT);

    SPIBusConfig busConfiguration;
    busConfiguration.clockDivider = SPI::ClockDivider::DIV_64;
    busConfiguration.mode         = SPI::Mode::MODE_3;

    VN100Spi sensor(bus, csPin, busConfiguration, 200);

    // Let the sensor start up
    Thread::sleep(1000);

    if (!sensor.init())
    {
        printf("Error, cannot initialize the sensor\n\n");
        return 0;
    }
    printf("Sensor initialized\n");

    if (!sensor.selfTest())
    {
        printf("Error while performing self test\n\n");
        return 0;
    }
    printf("Self test successful\n");

    for (int i = 0; i < 100; ++i)
    {
        sensor.sample();
        VN100SpiData sample = sensor.getLastSample();

        printf("sample %d:\n", i + 1);
        printf("acc: %llu, %.3f, %.3f, %.3f\n", sample.accelerationTimestamp,
               sample.accelerationX, sample.accelerationY,
               sample.accelerationZ);
        printf("ang: %.3f, %.3f, %.3f\n", sample.angularSpeedX,
               sample.angularSpeedY, sample.angularSpeedZ);
        printf("mag: %.3f, %.3f, %.3f\n", sample.magneticFieldX,
               sample.magneticFieldY, sample.magneticFieldZ);
        printf("quat: %.3f, %.3f, %.3f, %.3f\n\n", sample.quaternionX,
               sample.quaternionY, sample.quaternionZ, sample.quaternionW);

        Thread::sleep(500);
    }

    return 0;
}
