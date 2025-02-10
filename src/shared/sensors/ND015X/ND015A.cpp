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
    uint8_t* data;
    NDD015ADataExtended.pressure =
        (modeByte << 8) | rateByte;  // updating the first 2 bytes with the
                                     // correct sensor settings

    SPITransaction spi(slave);

    data = reinterpret_cast<uint8_t*>(&NDD015ADataExtended);

    spi.transfer(data, sizeof(NDD015ADataExtended));

    // check if the model returned by the sensor matches with the correct model
    for (int i = 4; i < 10; i++)
    {
        if (static_cast<uint8_t>(data[i]) !=
            static_cast<uint8_t>(sensorModel[i - 4]))
        {
            LOG_ERR(logger, "sensor model number did not match");
            return false;
        }
    }

    return true;
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

    SPITransaction spi(slave);

    uint16_t SPIDataOut = (modeByte << 8) | rateByte;
    spi.transfer16(SPIDataOut);
}

void ND015A::setIOWatchdog(IOWatchdogEnable iow)
{
    modeByte = (modeByte & ~IO_WATCHDOG_MASK) | iow;

    SPITransaction spi(slave);

    uint16_t SPIDataOut = (modeByte << 8) | rateByte;
    spi.transfer16(SPIDataOut);
}

void ND015A::setBWLimitFilter(BWLimitFilter bwl)
{
    modeByte = (modeByte & ~BW_LIMIT_MASK) | bwl;

    SPITransaction spi(slave);

    uint16_t SPIDataOut = (modeByte << 8) | rateByte;
    spi.transfer16(SPIDataOut);
}

void ND015A::setNotch(NotchEnable ntc)
{
    modeByte = (modeByte & ~NOTCH_MASK) | ntc;

    SPITransaction spi(slave);

    uint16_t SPIDataOut = (modeByte << 8) | rateByte;
    spi.transfer16(SPIDataOut);
}

ND015XData ND015A::sampleImpl()
{
    ND015XData data;
    uint16_t SPIDataOut = (modeByte << 8) | rateByte;

    SPITransaction spi(slave);

    uint16_t SPIDataIn = spi.transfer16(SPIDataOut);

    data.pressure =
        ((short)SPIDataIn - 0.05 * pow(2, 16)) / (0.9 * pow(2, 16)) * 15;
    data.pressureTimestamp = TimestampTimer::getTimestamp();

    return data;
}

}  // namespace Boardcore
