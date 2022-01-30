/* Copyright (c) 2019 Skyward Experimental Rocketry
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
#include <ostream>
#include <string>

namespace Boardcore
{

// Add here unique identifiers for threads in skyward-boardcore
enum ThreadId : uint8_t
{
    THID_MAV_RECEIVER,
    THID_MAV_SENDER,
    THID_XBEE,
    THID_GPS,
    THID_EVT_BROKER,
    THID_LOGGER_PACK,
    THID_LOGGER_WRITE,
    THID_CPU_METER,
    THID_CPU_WD,
    THID_PIN_OBS,

    // First available id to be used by repos that include boardcore as a
    // library
    THID_FIRST_AVAILABLE_ID
};

struct StackData
{
    long long timestamp       = 0;
    uint8_t threadId          = 0;
    unsigned int minimumStack = 0;

    static std::string header() { return "timestamp,threadId,minimumStack\n"; }

    void print(std::ostream& os)
    {
        os << timestamp << "," << (int)threadId << "," << minimumStack << "\n";
    }
};

}  // namespace Boardcore
