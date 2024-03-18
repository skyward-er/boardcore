/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Fabrizio Monti
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

#include "VN100Data.h"

namespace Boardcore
{

/**
 * @brief Driver class for VN100 Spi IMU.
 */
class VN100Spi : public Sensor<VN100Data>
{
public:
    /**
     * @brief VN100 constructor.
     *
     * @param bus SPI bus.
     * @param csPin SPI chip select pin.
     * @param busConfiguration SPI bus configuration.
     */
    VN100Spi(SPIBus& bus, miosix::GpioPin csPin, SPIBusConfig busConfiguration);

    /**
     * @brief Initialize the sensor.
     */
    bool init() override;

    /**
     * @brief Performs self test for the sensor.
     *
     * @return Return true if the test was successful.
     */
    bool selfTest() override;

    /**
     * @brief Gather data from the sensor.
     */
    VN100Data sampleImpl() override;

    // TODO: remove this, only for testing.
    AccelerometerData readAcc();

private:
    /**
     * @brief Check the model number register.
     *
     * @return Returns false if the red value is not valid.
     */
    bool checkModelNumber();

    /**
     * @brief Utility function used to clean the junk before starting to
     * communicate with the sensor. It send a 4 bytes packet of zeros to the
     * sensor.
     */
    void sendDummyPacket();

    /**
     * @brief Extracts floating point measurement from the data received from
     * the sensor.
     *
     * @param rawData The data received from the sensor.
     *
     * @return The floating point data.
     */
    float extractMeasurement(uint32_t rawData);

    bool isInit = false;

    SPISlave spiSlave;

    PrintLogger logger =
        Logging::getLogger("vn100-spi");  // TODO: is it fine? Should it be
                                          // changed to the exact model number?
};

}  // namespace Boardcore
