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

#include <reflect.hpp>

#include "PRF_types.h"

namespace Boardcore
{

struct WingControllerLogsData
{
    uint64_t timestamp;
    PRF_types_h_::PRFLogs PRFLogs;

    WingControllerLogsData() : timestamp(0), PRFLogs() {};

    WingControllerLogsData(uint64_t timestamp,
                           PRF_types_h_::PRFLogs wingControllerLogs)
        : timestamp(timestamp), PRFLogs(wingControllerLogs) {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            WingControllerLogsData,
            FIELD_DEF(timestamp) FIELD_DEF2(PRFLogs, Q1) FIELD_DEF2(PRFLogs, Q2)
                FIELD_DEF2(PRFLogs, TerminalTarget) FIELD_DEF2(
                    PRFLogs, TargetIndex) FIELD_DEF2(PRFLogs, Heading)
                    FIELD_DEF2(PRFLogs, Reference)
                        FIELD_DEF2(PRFLogs, ServoCommands)
                            FIELD_DEF2(PRFLogs, WindHeading));
    }
};

}  // namespace Boardcore
