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

#include <cmath>

namespace Boardcore
{
ND015A::ND015A(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig,
               IOWatchdogEnable iow = IOWatchdogEnable::DISABLED,
               BWLimitFilter bwl    = BWLimitFilter::BWL_200,
               NotchEnable ntc = NotchEnable::ENABLED, uint8_t odr = 0x1C)
    : slave(bus, cs, spiConfig), sensorSettings{0x7, iow, bwl, ntc, odr}
{
}

bool ND015A::init()
{
    ND015ADataExtended extendedData{};
    uint8_t* data = reinterpret_cast<uint8_t*>(&extendedData);

    // setting the first 2 bytes of the data to the correct sensor settings
    memcpy(&extendedData, &sensorSettings, sizeof(sensorSettings));

    SPITransaction spi(slave);
    spi.transfer(data, sizeof(extendedData));

    // this part checks if the model number returned by the sensor matches the
    // correct model number
    bool compareResult =
        (memcmp(&extendedData.model, &MODEL_NAME, sizeof(MODEL_NAME) - 1) == 0);

    if (!compareResult)
    {
        LOG_ERR(logger, "sensor model number did not match");
        return false;
    }
    return true;
}

bool ND015A::selfTest() { return true; }

void ND015A::setOutputDataRate(uint8_t odr)
{
    sensorSettings.odr = odr;

    SPITransaction spi(slave);
    uint16_t spiDataOut;

    memcpy(&spiDataOut, &sensorSettings, sizeof(spiDataOut));
    spi.transfer16(spiDataOut);
}

void ND015A::setIOWatchdog(IOWatchdogEnable iow)
{
    sensorSettings.iow = static_cast<uint8_t>(iow);

    SPITransaction spi(slave);
    uint16_t spiDataOut;

    memcpy(&spiDataOut, &sensorSettings, sizeof(spiDataOut));
    spi.transfer16(spiDataOut);
}

void ND015A::setBWLimitFilter(BWLimitFilter bwl)
{
    sensorSettings.bwl = static_cast<uint8_t>(bwl);

    SPITransaction spi(slave);
    uint16_t spiDataOut;

    memcpy(&spiDataOut, &sensorSettings, sizeof(spiDataOut));
    spi.transfer16(spiDataOut);
}

void ND015A::setNotch(NotchEnable ntc)
{
    sensorSettings.ntc = static_cast<uint8_t>(ntc);

    SPITransaction spi(slave);
    uint16_t spiDataOut;

    memcpy(&spiDataOut, &sensorSettings, sizeof(spiDataOut));
    spi.transfer16(spiDataOut);
}

ND015XData ND015A::sampleImpl()
{
    ND015XData data;
    uint16_t spiDataOut;
    SPITransaction spi(slave);

    memcpy(&spiDataOut, &sensorSettings, sizeof(spiDataOut));
    uint16_t spiDataIn = spi.transfer16(spiDataOut);

    data.pressure =
        ((short)spiDataIn - 0.05 * pow(2, 16)) / (0.9 * pow(2, 16)) * 15;
    data.pressureTimestamp = TimestampTimer::getTimestamp();

    return data;
}

}  // namespace Boardcore
