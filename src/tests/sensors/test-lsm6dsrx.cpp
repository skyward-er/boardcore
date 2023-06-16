/* Copyright (c) 2022 Skyward Experimental Rocketry
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
#include <miosix.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <utils/Debug.h>

using namespace Boardcore;
using namespace miosix;

int main()
{
    SPIBus bus(SPI3);

    GpioPin csPin(GPIOE_BASE, 3);  // PE3 CS
    csPin.mode(Mode::OUTPUT);

    GpioPin clockPin(GPIOB_BASE, 3);  // PB3 CK (SCL)
    clockPin.mode(Mode::ALTERNATE);
    clockPin.alternateFunction(6);
    GpioPin misoPin(GPIOB_BASE, 4);  // PB4 MISO (SDO)
    misoPin.alternateFunction(6);
    misoPin.mode(Mode::ALTERNATE);
    GpioPin mosiPin(GPIOB_BASE, 5);  // PB5 MOSI (SDA)
    mosiPin.alternateFunction(6);
    mosiPin.mode(Mode::ALTERNATE);

    SPIBusConfig busConfiguration;  // Bus configuration for the sensor
    busConfiguration.clockDivider = SPI::ClockDivider::DIV_256;
    busConfiguration.mode =
        SPI::Mode::MODE_0;  // Set clock polarity to 0 and phase to 1

    LSM6DSRXConfig sensConfig;
    sensConfig.bdu = LSM6DSRXConfig::BDU::CONTINUOUS_UPDATE;
    sensConfig.fsAcc = LSM6DSRXConfig::ACC_FULLSCALE::G2;
    sensConfig.odrAcc = LSM6DSRXConfig::ACC_ODR::HZ_104;
    sensConfig.opModeAcc = LSM6DSRXConfig::OPERATING_MODE::NORMAL;

    LSM6DSRX sens(bus, csPin, busConfiguration, sensConfig);

    if (sens.init() == false)
    {
        while (true)
        {
            TRACE("Error, sensor not initialized\n\n");
            Thread::sleep(1000);
        }
    }

    LSM6DSRX::AccData data{0.0, 0.0, 0.0};
    while (true)
    {
        sens.getAccelerometerData(data);
        TRACE("x: %f\n", data.x);
        TRACE("y: %f\n", data.y);
        TRACE("z: %f\n\n\n", data.z);

        Thread::sleep(1000);
    }

    return 0;
}