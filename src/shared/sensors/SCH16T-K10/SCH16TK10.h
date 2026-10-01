/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Tommaso Lamon
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

#include "SCH16TK10Data.h"

namespace Boardcore
{

template <typename Frame>
class SCH16TK10 : public Sensor<SCH16TK10Data>
{
public:
    /**
     * @brief Class constructor
     * @param bus SPI Bus Interface
     * @param cs Chip Select pin number
     * @param busConfig SPI Bus config parameters
     */
    SCH16TK10(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig busConfig);

    /**
     * Method to initialize the sensor
     */
    bool init() override;

    /**
     * Self-testing the sensor by writing and reading the SelfTest Register
     */
    bool selfTest() override;

    SCH16TK10Data sampleImpl() override;

    /**
     * Check the Whoami value. For this particular sensor the Whoami value
     * corresponds to the
     */
    bool checkWhoAmI();

private:
    PrintLogger logger = getLogger("sch16tk10");

    bool isInit = false;

    uint8_t calculateCRC(Frame::raw_t SPIFrame)
    {
        uint8_t result = Frame::calculateCRC(SPIFrame);
        return result;
    }
};
}  // namespace Boardcore
