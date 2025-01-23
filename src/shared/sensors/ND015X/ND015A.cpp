/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Pietro Bortolus
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

#include "ND015A.h"

#include <drivers/timer/TimestampTimer.h>
#include <math.h>

namespace Boardcore
{
ND015A::ND015A(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig)
    : slave(bus, cs, spiConfig)
{
}

bool ND015A::init()
{
    uint8_t data[10];

    SPITransaction spi(slave);

    spi.read(data, sizeof(data));

    // the following monstrosity is needed to check if the model number read
    // from the SPI is correct, the numbers are the ASCII encoding of "ND015A"

    if (data[9] == 0x41 && data[8] == 0x35 && data[7] == 0x31 &&
        data[6] == 0x30 && data[5] == 0x44 && data[4] == 0x4E)
    {
        return true;
    }
    else
    {
        LOG_ERR(logger, "sensor model number did not match");
        return false;
    }
}

bool ND015A::selfTest() { return true; }

void ND015A::setOutputDataRate(uint8_t odr)
{
    if (odr < 0x100)
    {
        rateByte = odr;
    }
    else
    {
        LOG_ERR(logger, "odr setting not valid, using default value (0x1C)");
        rateByte = 0x1C;
    }
}

void ND015A::setIOWatchdog(IOWatchdogEnable iow)
{
    modeByte = (modeByte & ~IO_WATCHDOG_MASK) | iow;
}

void ND015A::setBWLimitFilter(BWLimitFilter bwl)
{
    modeByte = (modeByte & ~BW_LIMIT_MASK) | bwl;
}

void ND015A::setNotch(NotchEnable ntc)
{
    modeByte = (modeByte & ~NOTCH_MASK) | ntc;
}

ND015XData ND015A::sampleImpl()
{
    ND015XData data;
    SPIDataOut = (modeByte << 8) | rateByte;

    SPITransaction spi(slave);

    spi.transfer16(SPIDataOut);  // we need to make an SPI transaction before
                                 // reading the data to make sure the proper
                                 // settings are used

    SPIDataIn = spi.transfer16(SPIDataOut);

    data.pressure =
        ((short)SPIDataIn - 0.05 * pow(2, 16)) / (0.9 * pow(2, 16)) * 15;
    data.pressureTimestamp = TimestampTimer::getTimestamp();

    return data;
}

}  // namespace Boardcore
