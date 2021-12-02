/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <Common.h>
#include <sensors/MAX6675/MAX6675.h>

using namespace miosix;

int main()
{
    //Creation of SPI bus
    SPIBus bus(SPI1);
    //Creation of the GPIO pins
    GpioPin spi_sck(GPIOA_BASE, 5);
    GpioPin spi_miso(GPIOA_BASE, 6);
    GpioPin cs(GPIOA_BASE, 3);

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  // Enable SPI1 bus

    //Setting the mode and the alternate function of the pins
    spi_sck.mode(Mode::ALTERNATE);
    spi_sck.alternateFunction(5);
    spi_miso.mode(Mode::ALTERNATE);
    spi_miso.alternateFunction(5);
    //Except cs which is the activation pin
    cs.mode(Mode::OUTPUT);
    //I put on high the cs pin
    cs.high();

    //Enable the timestamp
    TimestampTimer::enableTimestampTimer();

    //Sensor creation
    MAX6675 sensor{bus, cs};

    printf("Starting process verification!\n");

    if(!sensor.init())
    {
        printf("Sensor init failed!\n");
        return 0;
    }

    if(!sensor.selfTest())
    {
        printf("Sensor self test failed!\n");
        return 0;
    }

    for(int i = 0; i < 100; i++)
    {
        sensor.sample();
        TemperatureData sample = sensor.getLastSample();

        printf("%.1f\n", sample.temp);

        Thread::sleep(100);
    }
    return 0;
}