/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include "AD5204.h"

namespace Boardcore
{

AD5204::AD5204(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig,
               Resistance resistance)
    : slave(bus, cs, spiConfig), resRange(resistance)
{
}

void AD5204::setResistance(Channel channel, uint32_t resistance)
{
    SPITransaction spi(slave);

    if (resistance > static_cast<uint32_t>(resRange))
        return;

    uint8_t value = resistance / static_cast<float>(resRange) * 255;

    spi.write(static_cast<uint16_t>(static_cast<uint16_t>(channel) | value));
}

}  // namespace Boardcore
