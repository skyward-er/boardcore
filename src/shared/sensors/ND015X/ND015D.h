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

class ND015D : public Sensor<ND015XData>
{
public:
    /**
     * The datasheet is unclear about the unit of measure,
     * it could be either psi or inH2O but I believe it's the latter
     */
    enum FullScaleRange : uint8_t
    {
        FS_1  = 0x02,  // 1.0  psi
        FS_2  = 0x03,  // 2.0  psi
        FS_4  = 0x04,  // 4.0  psi
        FS_5  = 0x05,  // 5.0  psi
        FS_10 = 0x06,  // 10.0 psi
        FS_15 = 0x07,  // 15.0 psi
    };

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

    ND015D(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig);

    bool init() override;

    bool selfTest() override;

    /**
     * @brief function to set the output data rate
     *
     * @param odr   output data rate for the sensor,
     *              the actual odr is calculated as
     *              444Hz / odr
     */
    void setOutputDataRate(u_int8_t odr);

    /**
     * @brief function to set the fullscale range
     *
     * @param fs   fullscale range, default is
     *             2.0 psi
     */
    void setFullScaleRange(FullScaleRange fs);

    /**
     * @brief function to enable the IO watchdog
     *
     * @param iow  setting
     */
    void setIOWatchdog(IOWatchdogEnable iow);

    void setBWLimitFilter(BWLimitFilter bwl);

    void setNotch(NotchEnable ntc);

protected:
    ND015XData sampleImpl() override;

private:
    SPISlave slave;
    uint8_t modeByte = 0xF3;  // settings for the mode control register
    uint8_t rateByte = 0x1C;  // settings for the rate control register
    short range      = 1;

    enum RegisterMask : uint8_t
    {
        FS_MASK          = 0x07,
        IO_WATCHDOG_MASK = 0x08,
        BW_LIMIT_MASK    = 0x70,
        NOTCH_MASK       = 0x80,
    };

    PrintLogger logger = Logging::getLogger("nd015d");
};

}  // namespace Boardcore

