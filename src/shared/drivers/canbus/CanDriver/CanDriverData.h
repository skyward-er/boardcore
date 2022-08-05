/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <cstdint>

namespace Boardcore
{

namespace Canbus
{

struct CanTXResult
{
    uint32_t seq;
    uint8_t mailbox;
    uint8_t txStatus;
    uint8_t tme;
    uint8_t errCode;
};

struct CanRXStatus
{
    uint8_t rxStatus;
    uint8_t fifo;
    uint8_t rxErrCounter = 0;
    uint8_t errCode;
    bool fifoOverrun;
    bool fifoFull;
};

struct CanPacket
{
    uint32_t timestamp = 0;

    uint32_t id;
    bool ext = false;  ///< Whether to use extended packet id

    bool rtr = false;

    uint8_t length = 0;

    uint8_t data[8];
};

struct CanRXPacket
{
    CanPacket packet;
    CanRXStatus status;
};

}  // namespace Canbus

}  // namespace Boardcore
