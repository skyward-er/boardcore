/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include "MAX31856.h"

#include <drivers/timer/TimestampTimer.h>

namespace Boardcore
{

MAX31856::MAX31856(SPIBusInterface& bus, miosix::GpioPin cs,
                   SPIBusConfig config, ThermocoupleType type)
    : slave(bus, cs, config), type(type)
{
}

SPIBusConfig MAX31856::getDefaultSPIConfig()
{
    SPIBusConfig spiConfig{};
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;
    spiConfig.mode         = SPI::Mode::MODE_1;
    spiConfig.writeBit     = SPI::WriteBit::INVERTED;
    return spiConfig;
}

bool MAX31856::init()
{
    SPITransaction spi{slave};

    // Set thermocouple type
    setThermocoupleType(type);

    // Enable continuous conversion mode
    spi.writeRegister(CR0, CR0_CMODE);

    return true;
}

bool MAX31856::selfTest()
{
    SPITransaction spi{slave};

    // Enable open-circuit detection
    spi.writeRegister(CR0, CR0_CMODE | CR0_OCFAULT_0);

    // Wait for detection
    // Detection takes 40ms, waiting more to be extra sure
    miosix::Thread::sleep(100);

    // Read fault register
    auto fault = spi.readRegister(SR);

    // Disable open-circuit detection
    spi.writeRegister(CR0, CR0_CMODE);

    return !(fault & SR_OPEN);
}

void MAX31856::setThermocoupleType(ThermocoupleType type)
{
    SPITransaction spi{slave};
    spi.writeRegister(CR1, static_cast<uint8_t>(type));
}

TemperatureData MAX31856::sampleImpl()
{
    int32_t sample;

    {
        SPITransaction spi{slave};
        sample = spi.readRegister24(LTCBH);
    }

    TemperatureData result{};
    result.temperatureTimestamp = TimestampTimer::getTimestamp();

    // Sign extension
    sample = sample << 8;
    sample = sample >> (8 + 5);

    // Convert the integer and decimal part separately
    result.temperature = static_cast<float>(sample) * 0.007812;

    return result;
}

TemperatureData MAX31856::readInternalTemperature()
{
    uint16_t sample[2];

    {
        SPITransaction spi{slave};
        spi.read16(sample, sizeof(sample));
    }

    TemperatureData result{};
    result.temperatureTimestamp = TimestampTimer::getTimestamp();

    // Extract data bits
    sample[1] = sample[1] >> 4;

    // Convert the integer and decimal part separately
    result.temperature = static_cast<float>(sample[1] >> 4);
    result.temperature += static_cast<float>(sample[1] & 0xF) * 0.0625;

    return result;
}

}  // namespace Boardcore
