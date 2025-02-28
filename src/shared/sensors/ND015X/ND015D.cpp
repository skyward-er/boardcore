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

#include "ND015D.h"

#include <cmath.h>
#include <drivers/timer/TimestampTimer.h>

#define SENSOR_MODEL_OFFSET 4
#define SENSOR_MODEL_LENGTH 6

namespace Boardcore
{
ND015D::ND015D(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig,
               Config config)
    : slave(bus, cs, spiConfig), configuration(config)
{
}

bool ND015D::init()
{
    applyConfig(configuration);

    uint8_t* data;
    memcpy(&NDD015ADataExtended, &sensorSettings,
           sizeof(sensorSettings));  // updating the first 2 bytes with the
                                     // correct sensor settings

    SPITransaction spi(slave);

    data = reinterpret_cast<uint8_t*>(&NDD015ADataExtended);

    spi.transfer(data, sizeof(NDD015ADataExtended));

    // check if the model returned by the sensor matches with the correct model
    for (int i = SENSOR_MODEL_OFFSET;
         i < SENSOR_MODEL_OFFSET + SENSOR_MODEL_LENGTH; i++)
    {
        if (static_cast<uint8_t>(data[i]) !=
            static_cast<uint8_t>(sensorModel[i - SENSOR_MODEL_OFFSET]))
        {
            LOG_ERR(logger, "sensor model number did not match");
            return false;
        }
    }

    return true;
}

bool ND015D::selfTest() { return true; }

bool ND015D::applyConfig(Config config)
{
    sensorSettings.fsr = config.fsr;
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

void ND015D::setOutputDataRate(uint8_t odr)
{
    if (odr < 0x100)
    {
        sensorSettings.odr = odr;
    }
    else
    {
        LOG_ERR(logger, "odr setting not valid, using default value (0x1C)");
        rateByte = 0x1C;
    }

    SPITransaction spi(slave);
    uint16_t SPIDataOut;

    memcpy(&SPIDataOut, &sensorSettings, sizeof(SPIDataOut));
    spi.transfer16(SPIDataOut);
}

void ND015D::setFullScaleRange(FullScaleRange fsr)
{
    sensorSettings.fsr = fsr;

    switch (fs)
    {
        case FS_1:
            range = 1;
            break;

        case FS_2:
            range = 2;
            break;

        case FS_4:
            range = 4;
            break;

        case FS_5:
            range = 5;
            break;

        case FS_10:
            range = 10;
            break;

        case FS_15:
            range = 15;
            break;

        default:
            break;
    }

    SPITransaction spi(slave);
    uint16_t SPIDataOut;

    memcpy(&SPIDataOut, &sensorSettings, sizeof(SPIDataOut));
    spi.transfer16(SPIDataOut);
}

void ND015D::setIOWatchdog(IOWatchdogEnable iow)
{
    sensorSettings.iow = iow;

    SPITransaction spi(slave);
    uint16_t SPIDataOut;

    memcpy(&SPIDataOut, &sensorSettings, sizeof(SPIDataOut));
    spi.transfer16(SPIDataOut);
}

void ND015D::setBWLimitFilter(BWLimitFilter bwl)
{
    sensorSettings.bwl = bwl;

    SPITransaction spi(slave);
    uint16_t SPIDataOut;

    memcpy(&SPIDataOut, &sensorSettings, sizeof(SPIDataOut));
    spi.transfer16(SPIDataOut);
}

void ND015D::setNotch(NotchEnable ntc)
{
    sensorSettings.ntc = ntc;

    SPITransaction spi(slave);
    uint16_t SPIDataOut;

    memcpy(&SPIDataOut, &sensorSettings, sizeof(SPIDataOut));
    spi.transfer16(SPIDataOut);
}

ND015XData ND015D::sampleImpl()
{
    ND015XData data;
    uint16_t SPIDataOut;
    SPITransaction spi(slave);

    memcpy(&SPIDataOut, &sensorSettings, sizeof(SPIDataOut));
    uint16_t SPIDataIn = spi.transfer16(SPIDataOut);

    data.pressure          = (short)SPIDataIn / (0.9 * pow(2, 15)) * range;
    data.pressureTimestamp = TimestampTimer::getTimestamp();

    return data;
}

}  // namespace Boardcore
