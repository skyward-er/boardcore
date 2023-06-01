/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Author: Nuno Barcellos
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

#include <sensors/MS5803/MS5803I2C.h>

using namespace Boardcore;
using namespace miosix;

GpioPin scl(GPIOA_BASE, 8);
GpioPin sda(GPIOC_BASE, 9);
/**
 * This test is intended to be run on the Death Stack X
 */

int main()
{
    printf("Starting...\n");

    I2C bus(I2C3, scl, sda);
    I2CDriver::I2CSlaveConfig slaveConfig{0x76, I2CDriver::Addressing::BIT7,
                                          I2CDriver::Speed::STANDARD};

    while (true)
    {
        uint8_t reg = 0xD0;
        uint8_t data;
        if (bus.readRegister(slaveConfig, reg, data))
            printf("data: 0x%02x\n", data);

        delayMs(100);
    }
    // printf("2...\n");
    // MS5803I2C sensor(bus, 10);

    // printf("3...\n");

    // if (!sensor.init())
    // {
    //     printf("MS5803 Init failed\n");
    // }

    // printf("pressureTimestamp,press,temperatureTimestamp,temp\n");

    // while (true)
    // {
    //     sensor.sample();

    //     MS5803Data data = sensor.getLastSample();
    //     printf("%llu,%f,%llu,%f\n", data.pressureTimestamp, data.pressure,
    //            data.temperatureTimestamp, data.temperature);

    //     Thread::sleep(20);
    // }
}
