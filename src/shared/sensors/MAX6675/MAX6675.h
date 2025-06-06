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
 * @brief MAX6675 thermocouple sensor driver.
 */
class MAX6675 : public Sensor<TemperatureData>
{
public:
    MAX6675(SPIBusInterface& bus, miosix::GpioPin cs,
            SPIBusConfig config = getDefaultSPIConfig());

    static SPIBusConfig getDefaultSPIConfig();

    bool init();

    /**
     * @brief Checks whether the thermocouple is connected or not.
     *
     * @return True if the thermocouple is connected.
     */
    bool selfTest();

    /**
     * @brief Checks whether the thermocouple is connected or not.
     */
    bool checkConnection();

protected:
    TemperatureData sampleImpl() override;

private:
    const SPISlave slave;

    PrintLogger logger = Logging::getLogger("max6675");
};

}  // namespace Boardcore
