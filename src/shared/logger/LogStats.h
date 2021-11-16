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
 * Statistics for the logger
 */
class LogStats
{
public:
    /**
     * Constructor
     */
    LogStats() : timestamp(0) {}

    /**
     * Set timestamp for this class
     * \param timestamp timestamp
     */
    void setTimestamp(long long timestamp) { this->timestamp = timestamp; }

    static std::string header()
    {
        return "timestamp,logNumber,stat_toolarge,stat_dropped,stat_queued,stat_buf_"
               "filled,stat_buf_written,stat_w_failed,stat_w_time,stat_max_"
               "time,stat_last_error\n";
    }
    /**
     * Print the class fields to an ostream.
     * Used by the program that decodes the logged data after the flight.
     * \param os ostream where to print the class fields
     */
    void print(std::ostream& os) const
    {
        os << timestamp << "," << logNumber << "," << statTooLargeSamples << ","
           << statDroppedSamples << "," << statQueuedSamples << ","
           << statBufferFilled << "," << statBufferWritten << ","
           << statWriteFailed << "," << statWriteTime << "," << statMaxWriteTime
           << "," << statWriteError << "\n";
    }

    long long timestamp;  ///< Timestamp

    bool opened = false;
    int logNumber = 0;
    int statTooLargeSamples =
        0;  ///< Number of dropped samples because too large
    int statDroppedSamples = 0;  ///< Number of dropped samples due to fifo full
    int statQueuedSamples  = 0;  ///< Number of samples written to buffer
    int statBufferFilled   = 0;  ///< Number of buffers filled
    int statBufferWritten  = 0;  ///< Number of buffers written to disk
    int statWriteFailed    = 0;  ///< Number of fwrite() that failed
    int statWriteError     = 0;  ///< Error of the last fwrite() that failed
    int statWriteTime      = 0;  ///< Time to perform an fwrite() of a buffer
    int statMaxWriteTime = 0;  ///< Max time to perform an fwrite() of a buffer
};

}  // namespace Boardcore
