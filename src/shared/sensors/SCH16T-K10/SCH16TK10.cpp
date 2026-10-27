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

template <typename Frame>
typename Frame::raw_t SCH16TK10<Frame>::transferFrame(
    typename Frame::raw_t frame)
{
    constexpr BUFFER_SIZE = 8 / Frame::bits;
    uint8_t buf[BUFFER_SIZE];

    for (int i = 0; i < BUFFER_SIZE; i++)
        buf[i] = static_cast<uint8_t>(frame >> (8 * (BUFFER_SIZE - 1 - i)));

    {
        SPITransaction spiTransaction{spiSlave};
        spiTransaction.transfer(buf, sizeof(buf));
    }

    typename Frame::raw_t data = 0;
    for (int i = 0; i < BUFFER_SIZE; i++)
        data = (data << 8) | buf[i];

    return out;
}

template <typename Frame>
void SCH16TK10<Frame>::writeRegister(SCH16TK10Defs::Register target,
                                     uint32_t data)
{
    auto frame = Frame::encode(target, SCH16TK10Defs::Mode::WRITE, data);

    SPITransaction spiTransaction{spiSlave};

    if constexpr (Frame::bits == 32)
        spiTransaction.write32(frame);
    else if constexpr (Frame::bits == 48)
        spiTransaction.write48(frame);
}

template <typename Frame>
typename Frame::Response SCH16TK10<Frame>::readRegister(
    SCH16TK10Defs::Register target)
{
    auto rawData =
        transferFrame(Frame::encode(target, SCH16TK10Defs::Mode::READ, 0));

    Frame::Response resp = Frame::decode(rawData);
    return resp;
}

template <typename Frame>
bool SCH16TK10<Frame>::checkWhoAmI()
{
    auto resp = readRegister(SCH16TK10Defs::Register::COMP_ID);
    return resp.checkCRC && resp.data == SCH16TK10Defs::WHOAMI;
}

template <typename Frame>
bool SCH16TK10<Frame>::init()
{

}

template<typename Frame>
bool SCH16TK10<Frame>::testWrite() {
    writeRegister(SCH16TK10Defs::Register::SYS_TEST, 1);

    Frame::Response resp = readRegister(SCH16TK10Defs::Register::SYS_TEST)
    if (resp.data != 1)
        return false
    
    writeRegister(SCH16TK10Defs::Register::SYS_TEST, 0);

    Frame::Response resp = readRegister(SCH16TK10Defs::Register::SYS_TEST)
    if (resp.data != 0)
        return false
    
    return true;    
}


// TODO Finish
template <typename Frame>
bool SCH16TK10<Frame>::checkStatus() {
    Frame::Response resp = readRegister();
}

template <typename Frame>
bool SCH16TK10<Frame>::selfTest() {
    assert(!isInit && "[SCH16TK10] Need to init the sensor first.");

    isInit = 0;

    if (!testWrite() || !checkWhoAmI() || !checkStatus())
        return false;

    return init();
}

template class SCH16TK10<SPIFrame32>;
template class SCH16TK10<SPIFrame48>;
}  // namespace Boardcore
