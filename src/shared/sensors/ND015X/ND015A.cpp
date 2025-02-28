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

#include <cmath.h>
#include <drivers/timer/TimestampTimer.h>

#define SENSOR_MODEL_OFFSET 4

namespace Boardcore
{
ND015A::ND015A(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig,
               Config config)
    : slave(bus, cs, spiConfig), configuration(config)
{
}

bool ND015A::init()
{
    applyConfig(configuration);

    nd015aDataExtended extendedData;
    uint8_t* data = reinterpret_cast<uint8_t*>(&extendedData);

    // setting the first 2 bytes of the data to the correct sensor settings
    memcpy(&extendedData, &sensorSettings, sizeof(sensorSettings));

    SPITransaction spi(slave);
    spi.transfer(data, sizeof(extendedData));

    // this part checks if the model number returned by the sensor matches the
    // correct model number
    bool compareResult = (memcmp(&extendedData.model,
                                 &MODEL_NAME, sizeof(MODEL_NAME) - 1) == 0);

    if (!compareResult)
    {
        LOG_ERR(logger, "sensor model number did not match");
        return false;
    }
    return true;
}

bool ND015A::selfTest() { return true; }

bool ND015A::applyConfig(Config config)
{
    sensorSettings.iow = config.iow;
    sensorSettings.bwl = config.bwl;
    sensorSettings.ntc = config.ntc;

    if (config.odr < 0x100)
    {
        sensorSettings.odr = config.odr;
    }
    else
    {
        LOG_ERR(logger, "odr setting not valid, using default value (0x1C)");
        sensorSettings.odr = 0x1C;
    }

    SPITransaction spi(slave);
    uint16_t SPIDataOut;

    memcpy(&SPIDataOut, &sensorSettings, sizeof(SPIDataOut));
    spi.transfer16(SPIDataOut);

    return true;
}

void ND015A::setOutputDataRate(uint8_t odr)
{
    if (odr < 0x100)
    {
        sensorSettings.odr = odr;
    }
    else
    {
        LOG_ERR(logger, "odr setting not valid, using default value (0x1C)");
        sensorSettings.odr = 0x1C;
    }

    SPITransaction spi(slave);
    uint16_t SPIDataOut;

    memcpy(&SPIDataOut, &sensorSettings, sizeof(SPIDataOut));
    spi.transfer16(SPIDataOut);
}

void ND015A::setIOWatchdog(IOWatchdogEnable iow)
{
    sensorSettings.iow = iow;

    SPITransaction spi(slave);
    uint16_t SPIDataOut;

    memcpy(&SPIDataOut, &sensorSettings, sizeof(SPIDataOut));
    spi.transfer16(SPIDataOut);
}

void ND015A::setBWLimitFilter(BWLimitFilter bwl)
{
    sensorSettings.bwl = bwl;

    SPITransaction spi(slave);
    uint16_t SPIDataOut;

    memcpy(&SPIDataOut, &sensorSettings, sizeof(SPIDataOut));
    spi.transfer16(SPIDataOut);
}

void ND015A::setNotch(NotchEnable ntc)
{
    sensorSettings.ntc = ntc;

    SPITransaction spi(slave);
    uint16_t SPIDataOut;

    memcpy(&SPIDataOut, &sensorSettings, sizeof(SPIDataOut));
    spi.transfer16(SPIDataOut);
}

ND015XData ND015A::sampleImpl()
{
    ND015XData data;
    uint16_t SPIDataOut;
    SPITransaction spi(slave);

    memcpy(&SPIDataOut, &sensorSettings, sizeof(SPIDataOut));
    uint16_t SPIDataIn = spi.transfer16(SPIDataOut);

    data.pressure =
        ((short)SPIDataIn - 0.05 * pow(2, 16)) / (0.9 * pow(2, 16)) * 15;
    data.pressureTimestamp = TimestampTimer::getTimestamp();

    return data;
}

}  // namespace Boardcore
