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
#include "SCH16TK10.h"

#include "SCH16TK10Frames.h"

namespace Boardcore
{
using SCH16TK10Frames::SPIFrame32;
using SCH16TK10Frames::SPIFrame48;

bool SCH16TK10<SPIFrame48>::checkWhoAmI() {}

bool SCH16TK10<SPIFrame32>::checkWhoAmI() {}

void SCH16TK10<SPIFrame32>::writeRegister(SCH16TK10Defs::Register target,
                                          uint32_t data)
{
    auto frame = SPIFrame32::encode(target, SCH16TK10Defs::Mode::WRITE, data);

    {
        miosix::Lock<miosix::FastMutex> lock{imuMutex};
        SPITransaction spiTransaction{spiSlave};
        spiTransaction.write32(frame);
    }
}

void SCH16TK10<SPIFrame48>::writeRegister(SCH16TK10Defs::Register target,
                                          uint32_t data)
{
    auto frame = SPIFrame48::encode(target, SCH16TK10Defs::Mode::WRITE, data);

    {
        miosix::Lock<miosix::FastMutex> lock{imuMutex};
        SPITransaction spiTransaction{spiSlave};
        spiTransaction.write48(data);
    }
}

float SCH16TK10<SPIFrame32>::readRegister(SCH16TK10Defs::Register target)
{
    auto frame = SPIFrame32::encode(target, SCH16TK10Defs::Mode::READ, 0);

    uint8_t buf[8] = {
        static_cast<uint8_t>(frame >> 24), static_cast<uint8_t>(frame >> 16),
        static_cast<uint8_t>(frame >> 8),  static_cast<uint8_t>(frame),
        static_cast<uint8_t>(0),           static_cast<uint8_t>(0),
        static_cast<uint8_t>(0),           static_cast<uint8_t>(0)};

    {
        miosix::Lock<miosix::FastMutex> lock{imuMutex};
        SPITransaction spiTransaction{spiSlave};
        spiTransaction.transfer(buf, sizeof(buf));
    }

    // Extract data from buffer
}

}  // namespace Boardcore
