
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

#pragma once

#include <diagnostic/PrintLogger.h>
#include <drivers/spi/SPIDriver.h>
#include <sensors/Sensor.h>

#include "ND015XData.h"

namespace Boardcore
{

class ND015A : public Sensor<ND015XData>
{
public:
    enum BWLimitFilter : uint8_t
    {
        BWL_1   = 0x00,  // 1.0 Hz
        BWL_2   = 0x10,  // 2.0 Hz
        BWL_5   = 0x20,  // 5.0 Hz
        BWL_10  = 0x30,  // 10  Hz
        BWL_20  = 0x40,  // 20  Hz
        BWL_50  = 0x50,  // 50  Hz
        BWL_100 = 0x60,  // 100 Hz
        BWL_200 = 0x70,  // 200 Hz
    };

    enum IOWatchdogEnable : uint8_t
    {
        DISABLED = 0x00,
        ENABLED  = 0x08,
    };

    enum NotchEnable : uint8_t
    {
        DISABLED = 0x00,
        ENABLED  = 0x80,
    };

    /**
     * @brief Constructor for the ND015A sensor.
     *
     * @param bus SPI bus interface.
     * @param cs Chip select GPIO pin.
     * @param spiConfig SPI bus configuration.
     */
    ND015A(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig);

    /**
     * @brief Initializes the sensor.
     *
     * @return True if model number matches, false otherwise.
     */
    bool init() override;

    /**
     * @brief Not implemented.
     *
     * @return Always returns true.
     */
    bool selfTest() override;

    /**
     * @brief function to set the output data rate
     *
     * @param odr   output data rate for the sensor,
     *              the actual odr is calculated as
     *              444Hz / odr.
     *              Allowed values are 0x00 to 0xFF,
     *              0x00 will select the auto-select rate mode
     */
    void setOutputDataRate(u_int8_t odr);

    /**
     * @brief function is not implemented as the sensor has a fixed fullscale
     */
    void setFullScaleRange();

    /**
     * @brief function to enable the IO watchdog
     *
     * @param iow  setting
     */
    void setIOWatchdog(IOWatchdogEnable iow);

    /**
     * @brief Sets the bandwidth limit filter for the sensor.
     *
     * @param bwl Bandwidth limit filter setting.
     */
    void setBWLimitFilter(BWLimitFilter bwl);

    /**
     * @brief Enables or disables the notch filter.
     *
     * @param ntc Notch filter setting.
     */
    void setNotch(NotchEnable ntc);

protected:
    ND015XData sampleImpl() override;

private:
    SPISlave slave;
    uint8_t modeByte   = 0xF7;  // settings for the mode control register
    uint8_t rateByte   = 0x1C;  // settings for the rate control register
    string sensorModel = "ND015A";

    struct
    {
        uint16_t pressure;
        uint16_t temperature;
        uint8_t model[8];
        uint8_t serial[4];
        uint8_t build[6];
    } NDD015ADataExtended = {0};

    enum RegisterMask : uint8_t
    {
        FS_MASK          = 0x07,
        IO_WATCHDOG_MASK = 0x08,
        BW_LIMIT_MASK    = 0x70,
        NOTCH_MASK       = 0x80,
    };

    PrintLogger logger = Logging::getLogger("nd015a");
};

}  // namespace Boardcore

