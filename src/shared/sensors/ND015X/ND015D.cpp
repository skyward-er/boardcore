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

#include <drivers/spi/SPITransactionDMA.h>
#include <drivers/timer/TimestampTimer.h>
#include <utils/Constants.h>
#include <utils/Numeric.h>

#include <cmath>
#include <string>

namespace Boardcore
{
const char ND015D::MODEL_NAME[] = "ND015D";

SPIBusConfig ND015D::getDefaultSPIConfig()
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

ND015D::ND015D(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig,
               DMAStreamGuard* streamRx, DMAStreamGuard* streamTx,
               std::chrono::nanoseconds timeout, FullScaleRange fsr,
               IOWatchdogEnable iow, BWLimitFilter bwl, NotchEnable ntc,
               uint8_t odr)
    : slave(bus, cs, spiConfig), range(rangeToPressure(fsr)),
      streamRx(streamRx), streamTx(streamTx), timeoutDma(timeout),
      sensorSettings{odr, fsr, iow, bwl, ntc}
{
}

ND015D::ND015D(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig,
               FullScaleRange fsr, IOWatchdogEnable iow, BWLimitFilter bwl,
               NotchEnable ntc, uint8_t odr)
    : slave(bus, cs, spiConfig), range(rangeToPressure(fsr)), streamRx(nullptr),
      streamTx(nullptr), timeoutDma(std::chrono::nanoseconds::zero()),
      sensorSettings{odr, fsr, iow, bwl, ntc}
{
}

bool ND015D::init()
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

bool ND015D::selfTest() { return true; }

bool ND015D::checkModelMatch()
{
    ND015DDataExtended extendedData{};
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

void ND015D::setOutputDataRate(uint8_t odr)
{
    sensorSettings.odr = odr;

    SPITransaction spi(slave);
    uint16_t spiDataOut;

    memcpy(&spiDataOut, &sensorSettings, sizeof(spiDataOut));
    spi.transfer16(spiDataOut);
}

void ND015D::setFullScaleRange(FullScaleRange fsr)
{
    sensorSettings.fsr = fsr;

    range = rangeToPressure(fsr);

    SPITransaction spi(slave);
    uint16_t spiDataOut;

    memcpy(&spiDataOut, &sensorSettings, sizeof(spiDataOut));
    spi.transfer16(spiDataOut);
}

float ND015D::rangeToPressure(FullScaleRange fsr)
{
    switch (fsr)
    {
        case ND015D::FullScaleRange::FS_1:
            return 1;
        case ND015D::FullScaleRange::FS_2:
            return 2;
        case ND015D::FullScaleRange::FS_4:
            return 4;
        case ND015D::FullScaleRange::FS_5:
            return 5;
        case ND015D::FullScaleRange::FS_10:
            return 10;
        case ND015D::FullScaleRange::FS_15:
            return 15;
        default:
            return 0;
    }
}

void ND015D::setIOWatchdog(IOWatchdogEnable iow)
{
    sensorSettings.iow = iow;

    SPITransaction spi(slave);
    uint16_t spiDataOut;

    memcpy(&spiDataOut, &sensorSettings, sizeof(spiDataOut));
    spi.transfer16(spiDataOut);
}

void ND015D::setBWLimitFilter(BWLimitFilter bwl)
{
    sensorSettings.bwl = bwl;

    SPITransaction spi(slave);
    uint16_t spiDataOut;

    memcpy(&spiDataOut, &sensorSettings, sizeof(spiDataOut));
    spi.transfer16(spiDataOut);
}

void ND015D::setNotch(NotchEnable ntc)
{
    sensorSettings.ntc = ntc;

    SPITransaction spi(slave);
    uint16_t spiDataOut;

    memcpy(&spiDataOut, &sensorSettings, sizeof(spiDataOut));
    spi.transfer16(spiDataOut);
}

void ND015D::setOffset(float offset) { pressureOffset = offset; };
void ND015D::updateOffset(float offset) { pressureOffset += offset; };
float ND015D::getOffset() { return pressureOffset; };

ND015XData ND015D::sampleImpl()
{
    ND015XData data;
    uint16_t spiDataOut;
    uint16_t spiDataIn = 0;

    memcpy(&spiDataOut, &sensorSettings, sizeof(spiDataOut));

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

    float normalizedPressure =
        std::bit_cast<int16_t>(spiDataIn) / (0.9f * powf(2, 15));

    data.pressure =
        normalizedPressure * range * Constants::PSI_TO_PASCAL + pressureOffset;

    data.pressureTimestamp = TimestampTimer::getTimestamp();

    return data;
}

}  // namespace Boardcore
