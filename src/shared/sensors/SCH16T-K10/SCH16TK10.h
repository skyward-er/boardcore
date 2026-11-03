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
#include <miosix.h>
#include <sensors/Sensor.h>

#include <type_traits>

#include "SCH16TK10Data.h"
#include "SCH16TK10Defs.h"
#include "SCH16TK10Frames.h"

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
    SCH16TK10(SPIBusInterface& bus, miosix::GpioPin cs,
              SPIBusConfig busConfig = getDefaultSPIConfig())
        : spiSlave(bus, cs, busConfig)
    {
        static_assert(
            std::is_same<Frame, SCH16TK10Frames::SPIFrame48>::value ||
                std::is_same<Frame, SCH16TK10Frames::SPIFrame32>::value,
            "SCH16TK10: Frame must be SPIFrame32 or SPIFrame48");
    };

    static SPIBusConfig getDefaultSPIConfig()
    {
        SPIBusConfig config{};

        config.clockDivider = SPI::ClockDivider::DIV_32;
        config.mode         = SPI::Mode::MODE_0;

        return config;
    }

    /**
     * @brief Method to initialize the sensor
     */
    bool init() override;

    /**
     * @brief Self-testing the sensor by writing and reading the SelfTest
     * Register
     */
    bool selfTest() override;

    SCH16TK10Data sampleImpl() override;

    /**
     * @brief Check the Whoami value. For this particular sensor the Whoami
     * value corresponds to the COMP_ID
     */
    bool checkWhoAmI();

private:
    bool isInit = false;

    /**
     * @brief Write a register of the sensor.
     * @param target Target register.
     * @param data Payload of the message.
     */
    void writeRegister(SCH16TK10Defs::Register target, uint32_t data);

    /**
     * @brief Read a register of the sensor.
     * @param target Target register.
     * @return Response struct containing data and additional info.
     */
    typename Frame::Response readRegister(SCH16TK10Defs::Register target);

    /**
     * @brief Helper method to transfer a frame.
     * @param frame Frame to send.
     * @return Frame received.
     */
    typename Frame::raw_t transferFrame(typename Frame::raw_t frame);

    /**
     * @brief Test writing a register.
     * @return Result of operations.
     */
    bool testWrite();

    /**
     * @brief Check sensor status upon initialization.
     * @return Result of operations.
     */
    bool checkStatus();

    /**
     * @brief Check sensor's config upon initialization.
     * @return Result of operations.
     */
    bool checkConfig();

    SPISlave spiSlave;
};
}  // namespace Boardcore
