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

#pragma once

#include <diagnostic/PrintLogger.h>
#include <drivers/spi/SPIDriver.h>
#include <sensors/Sensor.h>
#include <sensors/SensorData.h>

namespace Boardcore
{

/**
 * @brief MAX6675 termocouple sensor driver.
 */
class MAX6675 : public Sensor<TemperatureData>
{
public:
    /**
     * @brief Constructor.
     *
     * @param bus The Spi bus.
     * @param cs The CS pin to lower when we need to sample.
     * @param config The SPI configuration.
     */
    MAX6675(SPIBusInterface &bus, GpioPin cs,
            SPIBusConfig config = getDefaultSPIConfig());

    /**
     * Constructs the default config for SPI Bus.
     *
     * @returns The default SPIBusConfig.
     */
    static SPIBusConfig getDefaultSPIConfig();

    bool init();

    /**
     * @brief Checks wheter the termocouple is open.
     *
     * @return True if the termocouple is connected.
     */
    bool selfTest();

private:
    TemperatureData sampleImpl() override;

    SPISlave slave;

    PrintLogger logger = Logging::getLogger("max6675");
};

}  // namespace Boardcore