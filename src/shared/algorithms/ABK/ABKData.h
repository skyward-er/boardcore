/* Copyright (c) 2026 Skyward Experimental Rocketry
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

#include <abk/ABK_types.h>

#include <reflect.hpp>

namespace Boardcore
{

struct ABKLogsData
{
    uint64_t timestamp;
    ABK_types_h_::ABKLogs ABKLogs;

    ABKLogsData() : timestamp(0), ABKLogs() {};

    ABKLogsData(uint64_t timestamp, ABK_types_h_::ABKLogs abkLogs)
        : timestamp(timestamp), ABKLogs(abkLogs) {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            ABKLogsData, FIELD_DEF(timestamp) FIELD_DEF2(ABKLogs, ABKCommand)
                             FIELD_DEF2(ABKLogs, FilterCoefficient)
                                 FIELD_DEF2(ABKLogs, PrePIDCommand)
                                     FIELD_DEF2(ABKLogs, PostPIDCommand)
                                         FIELD_DEF2(ABKLogs, BypassActivation));
    }
};

}  // namespace Boardcore
