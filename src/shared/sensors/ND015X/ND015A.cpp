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

#include <drivers/spi/SPITransactionDMA.h>
#include <drivers/timer/TimestampTimer.h>
#include <utils/Constants.h>

#include <cmath>
#include <string>

namespace Boardcore
{
const char ND015A::MODEL_NAME[] = "ND015A";

SPIBusConfig ND015A::getDefaultSPIConfig()
{
    SPIBusConfig spiConfig{};
    spiConfig.mode         = SPI::Mode::MODE_1;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_256;

    // Datasheet specifies 100us, but 50us is enough from testing
    spiConfig.csSetupTimeUs = 50;

    // The datasheet specifies a minimum CS hold time of 100us
    // If this is ever an issue, implement it by waiting at the next sample
    // instead of setting it in SPIBusConfig

    return spiConfig;
}

ND015A::ND015A(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig,
               DMAStreamGuard* streamRx, DMAStreamGuard* streamTx,
               std::chrono::nanoseconds timeout, IOWatchdogEnable iow,
               BWLimitFilter bwl, NotchEnable ntc, uint8_t odr)
    : slave(bus, cs, spiConfig), streamRx(streamRx), streamTx(streamTx),
      timeoutDma(timeout), sensorSettings{0x7, iow, bwl, ntc, odr}
{
}

ND015A::ND015A(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig,
               IOWatchdogEnable iow, BWLimitFilter bwl, NotchEnable ntc,
               uint8_t odr)
    : slave(bus, cs, spiConfig), streamRx(nullptr), streamTx(nullptr),
      timeoutDma(std::chrono::nanoseconds::zero()),
      sensorSettings{0x7, iow, bwl, ntc, odr}
{
}

bool ND015A::init()
{
    // setting the sensor settings to the correct values
    SPITransaction spi(slave);
    uint16_t spiDataOut;

    memcpy(&spiDataOut, &sensorSettings, sizeof(spiDataOut));
    spi.transfer16(spiDataOut);

    // Using the sensor immediately after initialization results in incorrect
    // readings. To avoid this, we introduce a delay. The minimum delay required
    // to prevent errors is 400 microseconds, but for safety and reliability, we
    // set it to 4 milliseconds.
    miosix::Thread::sleep(4);

    return true;
}

bool ND015A::selfTest() { return true; }

bool ND015A::checkModelMatch()
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
        auto model =
            std::string(extendedData.model, sizeof(extendedData.model));
        // Replace all \0 with '.' for printing
        std::replace(model.begin(), model.end(), '\0', '.');

        LOG_ERR(logger,
                "Sensor model mismatch: received {}, expected {} (. = \\0)",
                model, MODEL_NAME);
        return false;
    }
    return true;
}

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
    sensorSettings.iow = iow;

    SPITransaction spi(slave);
    uint16_t spiDataOut;

    memcpy(&spiDataOut, &sensorSettings, sizeof(spiDataOut));
    spi.transfer16(spiDataOut);
}

void ND015A::setBWLimitFilter(BWLimitFilter bwl)
{
    sensorSettings.bwl = bwl;

    SPITransaction spi(slave);
    uint16_t spiDataOut;

    memcpy(&spiDataOut, &sensorSettings, sizeof(spiDataOut));
    spi.transfer16(spiDataOut);
}

void ND015A::setNotch(NotchEnable ntc)
{
    sensorSettings.ntc = ntc;

    SPITransaction spi(slave);
    uint16_t spiDataOut;

    memcpy(&spiDataOut, &sensorSettings, sizeof(spiDataOut));
    spi.transfer16(spiDataOut);
}

ND015XData ND015A::sampleImpl()
{
    ND015XData data;
    uint16_t spiDataOut;

    memcpy(&spiDataOut, &sensorSettings, sizeof(spiDataOut));

    uint16_t spiDataIn = 0;
    if (streamRx != nullptr && streamTx != nullptr)
    {
        // Use dma
        SPITransactionDMA spi(slave, *streamRx, *streamTx);
        spiDataIn = spi.transfer16(spiDataOut, timeoutDma);
    }
    else
    {
        SPITransaction spi(slave);
        spiDataIn = spi.transfer16(spiDataOut);
    }

    data.pressure =
        ((spiDataIn - 0.05 * pow(2, 16)) / (0.9 * pow(2, 16)) * 15) *
        Constants::PSI_TO_PASCAL;
    data.pressureTimestamp = TimestampTimer::getTimestamp();

    return data;
}

}  // namespace Boardcore
