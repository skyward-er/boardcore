/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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

#include <stdint.h>

namespace Boardcore
{

namespace Canbus
{

/**
 * @brief Generic struct that contains a can protocol message.
 *
 * For example an accelerometer message could have:
 * - 4 bytes for the timestamp
 * - 3x4 bytes for float values
 * This message would be divided into 2 can packets.
 *
 * Note that the maximum size for a message is 520 bytes since the remaining
 * packet information is 6 bit wide.
 */
struct CanMessage
{
    int32_t id     = -1;  ///< Id of the message without sequential infos.
    uint8_t length = 0;   ///< Length of the message content.
    uint64_t payload[65];
};

inline bool operator==(const CanMessage& lhs, const CanMessage& rhs)
{
    if (lhs.id != rhs.id || lhs.length != rhs.length)
        return false;

    for (int i = 0; i < lhs.length; i++)
        if (lhs.payload[i] != rhs.payload[i])
            return false;

    return true;
}

inline bool operator!=(const CanMessage& lhs, const CanMessage& rhs)
{
    return !(lhs == rhs);
}

}  // namespace Canbus

}  // namespace Boardcore
