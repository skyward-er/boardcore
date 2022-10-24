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
#include <sensors/MPU9250/MPU9250.h>
#include <utils/Debug.h>

using namespace miosix;

using namespace Boardcore;

GpioPin sckPin  = GpioPin(GPIOB_BASE, 13);
GpioPin misoPin = GpioPin(GPIOB_BASE, 14);
GpioPin mosiPin = GpioPin(GPIOB_BASE, 15);
GpioPin csPin   = GpioPin(GPIOC_BASE, 1);

void initBoard()
{
    // Alternate function configuration for SPI pins
    sckPin.mode(miosix::Mode::ALTERNATE);
    sckPin.alternateFunction(5);  // SPI function
    mosiPin.mode(miosix::Mode::ALTERNATE);
    mosiPin.alternateFunction(5);  // SPI function
    misoPin.mode(miosix::Mode::ALTERNATE);
    misoPin.alternateFunction(5);  // SPI function

    // Chip select pin as output starting high
    csPin.mode(miosix::Mode::OUTPUT);
    csPin.high();
}

int main()
{
    // Enable SPI clock and set gpios
    // initBoard();

    // Device initialization
    SPIBus spiBus(SPI1);
    MPU9250 mpu9250(spiBus, sensors::mpu9250::cs::getPin());

    // Initialize the device
    bool result = mpu9250.init();
    printf("Init result: %d\n", result);

    result = mpu9250.selfTest();
    printf("Self test result: %d\n", result);

    while (true)
    {
        mpu9250.sample();
        MPU9250Data data = mpu9250.getLastSample();
        printf("%lld,%f,%f,%f;", data.accelerationTimestamp, data.accelerationX,
               data.accelerationY, data.accelerationZ);
        printf("%lld,%f,%f,%f;", data.angularSpeedTimestamp, data.angularSpeedX,
               data.angularSpeedY, data.angularSpeedZ);
        printf("%lld,%f,%f,%f\n", data.magneticFieldTimestamp,
               data.magneticFieldX, data.magneticFieldY, data.magneticFieldZ);

        // Serial communication at 115200 baud takes approximately 10ms
        // miosix::delayMs(10);
    }

    TRACE("Test completed\n");
}
