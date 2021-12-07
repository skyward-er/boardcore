/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include "MAX6675.h"

namespace Boardcore
{

MAX6675::MAX6675(SPIBusInterface &bus, GpioPin cs, SPIBusConfig config)
    : slave(bus, cs, config)
{
}

SPIBusConfig MAX6675::getDefaultSPIConfig()
{
    SPIBusConfig spiConfig{};
    spiConfig.clock_div = SPIClockDivider::DIV32;
    spiConfig.mode      = SPIMode::MODE1;
    return spiConfig;
}

bool MAX6675::init() { return true; }

bool MAX6675::selfTest()
{
    uint16_t sample;

    {
        SPITransaction spi{slave};
        sample = spi.read(0x00);
    }

    // The third bit (D2) indicated wheter the termocouple is connected or not.
    // It is high if open
    if ((sample % 0x2) != 0)
    {
        last_error = SensorErrors::SELF_TEST_FAIL;
        LOG_ERR(logger, "Self test failed, the termocouple is not connected");
        return false;
    }

    return true;
}

TemperatureData MAX6675::sampleImpl()
{
    uint16_t sample;

    {
        SPITransaction spi{slave};
        sample = spi.read(0x00);
    }

    // Extract bits 14-3
    sample &= 0x7FF8;
    sample >>= 3;

    TemperatureData result{};
    result.temp_timestamp = TimestampTimer::getTimestamp();

    // Convert the sample value
    result.temp = static_cast<unsigned int>(sample >> 2);  // Integer part
    sample &= 0x0003;  // Take the floating point part
    result.temp += static_cast<float>(sample * 0.25);  // Floating point part

    return result;
}

}  // namespace Boardcore
