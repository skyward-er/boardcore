/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Author: Federico Terraneo
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

#include <ostream>
#include <reflect.hpp>
#include <string>

namespace Boardcore
{
/**
 * @brief Statistics for the logger.
 */
struct LoggerStats
{
    uint64_t timestamp = 0;

    int32_t logNumber = 0;

    ///< Number of dropped samples because they where too large.
    int32_t tooLargeSamples = 0;

    int32_t droppedSamples =
        0;  ///< Number of dropped samples due to fifo full.
    int32_t queuedSamples  = 0;  ///< Number of samples written to buffer.
    int32_t queuedMappings = 0;  ///< Number of mappings written to buffer.
    int32_t buffersFilled  = 0;  ///< Number of buffers filled.
    int32_t buffersWritten = 0;  ///< Number of buffers written to disk.
    int32_t writesFailed   = 0;  ///< Number of fwrite() that failed.
    int32_t lastWriteError = 0;  ///< Error of the last fwrite() that failed.
    int32_t averageWriteTime =
        0;                     ///< Average time for an fwrite() of a buffer.
    int32_t maxWriteTime = 0;  ///< Max time for an fwrite() of a buffer.

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            LoggerStats,
            FIELD_DEF(timestamp) FIELD_DEF(logNumber) FIELD_DEF(tooLargeSamples)
                FIELD_DEF(droppedSamples) FIELD_DEF(queuedSamples)
                    FIELD_DEF(queuedMappings) FIELD_DEF(buffersFilled)
                        FIELD_DEF(buffersWritten) FIELD_DEF(writesFailed)
                            FIELD_DEF(lastWriteError)
                                FIELD_DEF(averageWriteTime)
                                    FIELD_DEF(maxWriteTime));
    };
};
}  // namespace Boardcore

