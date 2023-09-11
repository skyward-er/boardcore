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

    // Reset the cold junction offset
    setColdJunctionOffset(0);

    // Enable continuous conversion mode
    spi.writeRegister(CR0, CR0_CMODE);

    return true;
}

bool MAX31856::selfTest() { return true; }

void MAX31856::setThermocoupleType(ThermocoupleType type)
{
    SPITransaction spi{slave};
    spi.writeRegister(CR1, static_cast<uint8_t>(type));
}

void MAX31856::setColdJunctionOffset(float offset)
{
    SPITransaction spi{slave};
    spi.writeRegister(CJTO,
                      static_cast<int8_t>(offset * 1 / CJ_TEMP_LSB_VALUE));
}

MAX31856Data MAX31856::sampleImpl()
{
    SPITransaction spi{slave};
    int16_t cjRaw = spi.readRegister16(CJTH);
    int32_t tcRaw = spi.readRegister24(LTCBH);

    MAX31856Data result;
    result.temperatureTimestamp = TimestampTimer::getTimestamp();

    // The register has:
    // - A leading sign bit (which is actually two's complement)
    // - The other 18 bits
    // - 5 unused trailing bits

    // Since the 24 bit registers value is stored into a 32 bit variable, first
    // we make a left shift to move the sign bit in the 31st bit, and then a
    // right shift to remove the unused bits.
    // This automatically extends the sign
    tcRaw = tcRaw << 8;
    tcRaw = tcRaw >> (8 + 5);

    // Here we just make a right shift as the sign bit is already in the 15th
    // bit. This will also perform the sign extension like in sampleImpl()
    cjRaw = cjRaw >> 4;

    // Convert the raw value into temperature
    result.temperature = static_cast<float>(tcRaw) * TC_TEMP_LSB_VALUE;

    // Convert the raw value into temperature
    result.coldJunctionTemperature =
        static_cast<float>(cjRaw) * CJ_TEMP_LSB_VALUE;

    return result;
}

}  // namespace Boardcore
