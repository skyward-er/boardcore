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

#include <drivers/spi/SPIDriver.h>
#include <sensors/Sensor.h>

#include "ND015XData.h"

namespace Boardcore
{

class ND015X : public Sensor<ND015XData>
{
public:
    enum OutputDataRate : uint16_t
    {
        ODR_50   = 0x00,
        ODR_100  = 0x08,
        ODR_400  = 0x10,
        ODR_1000 = 0x18,
    };

    enum FullScaleRange : uint8_t
    {
        FS_6  = 0x00,
        FS_12 = 0x10,
        FS_24 = 0x30,
    };

    ND015X(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig);

    bool init() override;

    bool selfTest() override;

    void setOutputDataRate(OutputDataRate odr);

    void setFullScaleRange(FullScaleRange fs);

protected:
    ND015XData sampleImpl() override;

private:
    SPISlave slave;
    uint16_t SPIDataIn;
    uint16_t SPIDataOut;
}

}  // namespace Boardcore
