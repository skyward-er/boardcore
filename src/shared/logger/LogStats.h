/***************************************************************************
 *   Copyright (C) 2018 by Terraneo Federico                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

#pragma once

#include <ostream>
#include <string>

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
