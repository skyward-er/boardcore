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
#pragma once

#include <algorithms/SDA/Kriging0_types.h>

#include <reflect.hpp>

namespace Boardcore
{

struct SDALogsWrapper
{
    Kriging0_types_h_::SDALogs logs;

    SDALogsWrapper() : logs() {};

    SDALogsWrapper(Kriging0_types_h_::SDALogs logs) : logs(logs) {};

    SDALogsWrapper(Kriging0_types_h_::SDALogs logs, uint64_t timestamp)
        : logs(logs)
    {
        this->logs.Timestamp = timestamp;
    };

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            SDALogsWrapper,
            FIELD_DEF2(logs, ShutdownCommand) FIELD_DEF2(logs, Timestamp)
                FIELD_DEF2(logs, ShutdownCounter) FIELD_DEF2(logs, Apogee));
    }
};

};  // namespace Boardcore
