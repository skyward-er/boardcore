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
#include <string>

namespace Boardcore
{

/**
 * @brief Statistics for the logger.
 */
struct LoggerStats
{
    uint64_t timestamp = 0;

    int logNumber = 0;

    ///< Number of dropped samples because they where too large.
    int tooLargeSamples = 0;

    int droppedSamples   = 0;  ///< Number of dropped samples due to fifo full.
    int queuedSamples    = 0;  ///< Number of samples written to buffer.
    int buffersFilled    = 0;  ///< Number of buffers filled.
    int buffersWritten   = 0;  ///< Number of buffers written to disk.
    int writesFailed     = 0;  ///< Number of fwrite() that failed.
    int lastWriteError   = 0;  ///< Error of the last fwrite() that failed.
    int averageWriteTime = 0;  ///< Average time for an fwrite() of a buffer.
    int maxWriteTime     = 0;  ///< Max time for an fwrite() of a buffer.

    static std::string header()
    {
        return "timestamp,logNumber,tooLargeSamples,droppedSamples,"
               "queuedSamples,buffersFilled,buffersWritten,writesFailed,"
               "lastWriteError,averageWriteTime,maxWriteTime\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << logNumber << "," << tooLargeSamples << ","
           << droppedSamples << "," << queuedSamples << "," << buffersFilled
           << "," << buffersWritten << "," << writesFailed << ","
           << lastWriteError << "," << averageWriteTime << "," << maxWriteTime
           << "\n";
    }
};

}  // namespace Boardcore
