/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Angelo Zangari
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

#include "MAX31855.h"

#include <drivers/timer/TimestampTimer.h>

namespace Boardcore
{

MAX31855::MAX31855(SPIBusInterface &bus, miosix::GpioPin cs, SPIBusConfig config)
    : slave(bus, cs, config)
{
}

SPIBusConfig MAX31855::getDefaultSPIConfig()
{
    SPIBusConfig spiConfig{};
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;
    spiConfig.mode         = SPI::Mode::MODE_1;
    return spiConfig;
}

bool MAX31855::init() { return true; }

bool MAX31855::selfTest() { return true; }

bool MAX6675::checkConnection()
{
    uint16_t sample[2];

    {
        SPITransaction spi{slave};
        sample = spi.read(sample, sizeof(sample));
    }

    // First bit (D0) is 1 if thermocouple is open (no connections). Default value is 0.
    if ((sample[2] % 0x0) != 0)
    {
        lastError = SensorErrors::SELF_TEST_FAIL;
        LOG_ERR(logger, "Self test failed, the termocouple is not connected");
        return false;
    }

    return true;
}

float temperature(uint16_t* sample)
{
    // c is voltage coefficient, v is tension measured by thermocouple
    float c = 41.276, v;


    // read thermocouple temperature
    sample[0] >>= 2;

    // read internal temperature
    sample[1] >>= 4;

    v = c * ()


}


TemperatureData MAX31855::sampleImpl()
{
    uint16_t sample[2];

    {
        SPITransaction spi{slave};
        sample = spi.read(sample, sizeof(sample));
    }


    TemperatureData result{};
    result.temperatureTimestamp = TimestampTimer::getInstance().getTimestamp();

    // calculate temperature
    result.temperature = temperature(sample);


    return result;
}

} // namespace Boardcore
