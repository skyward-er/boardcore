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

#include <drivers/spi/SPIDriver.h>

namespace Boardcore
{

class AD5204
{
public:
    enum class Channel : uint16_t
    {
        RDAC_1 = 0x000,
        RDAC_2 = 0x100,
        RDAC_3 = 0x200,
        RDAC_4 = 0x300,
    };

    enum class Resistance : uint32_t
    {
        R_10  = 10000,
        R_50  = 50000,
        R_100 = 100000,
    };

    AD5204(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig,
           Resistance resistance = Resistance::R_10);

    void setResistance(Channel channel, uint32_t resistance);

private:
    SPISlave slave;
    Resistance resRange;
};

}  // namespace Boardcore
