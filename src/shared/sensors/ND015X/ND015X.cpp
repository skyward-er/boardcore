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

#include "ND015X.h"

#include <drivers/timer/TimestampTimer.h>

namespace Boardcore
{
ND015X::ND015X(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig)
    : slave(bus, cs, spiConfig)
{
}

bool ND015X::init() { return true; }

bool ND015X::selfTest() {}

void ND015X::setOutputDataRate(OutputDataRate odr)
{
    SPIDataIn = (SPIDataIn & (255 << 8)) | odr;
}

void ND015X::setFullScaleRange(FullScaleRange fs)
{
    SPIDataIn = (SPIDataIn & 255) | (fs << 8);
}

ND015XData ND015X::sampleImpl()
{
    ND015XData data;

    SPITransaction spi(slave);

    SPIDataOut = spi.transfer16(SPIDataOut);

    return data;
}

}  // namespace Boardcore
