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

#include <miosix.h>
#include <cstdio>
#include <list>
#include <queue>
#include <string>
#include <type_traits>
#include "LogStats.h"

using std::string;
/**
 * Possible outcomes of Logger::log()
 */
enum class LogResult
{
    Queued,   ///< Data has been accepted by the logger and will be written
    Dropped,  ///< Buffers are currently full, data will not be written. Sorry
    Ignored,  ///< Logger is currently stopped, data will not be written
    TooLarge  ///< Data is too large to be logged. Increase maxRecordSize
};

/**
 * Buffered logger. Needs to be started before it can be used.
 */
class Logger
{
public:
    /**
     * \return an instance to the logger
     */
    static Logger &instance();

    /**
     * Blocking call. May take a long time.
     *
     * Call this function to start the logger.
     * When this function returns, the logger is started, and subsequent calls
     * to log will actually log the data.
     *
     * \throws runtime_error if the log could not be opened
     * \return log number
     */
    int start();

    /**
     * Blocking call. May take a very long time (seconds).
     *
     * Call this function to stop the logger.
     * When this function returns, all log buffers have been flushed to disk,
     * and it is safe to power down the board without losing log data or
     * corrupting the filesystem.
     */
    void stop();

    /**
     * Return the number representing the current log file.
     * @return log number
     */
    int getLogNumber() { return fileNumber; }

    /**
     * Returns the log filename for the specified number.
     * IE: log_number = 16, returned: "/sd/log16.dat"
     * @param log_number
     * @return
     */
    static string getFileName(int log_number)
    {
        char filename[32];
        sprintf(filename, "/sd/log%02d.dat", log_number);

        return string(filename);
    }

    /**
    * Returns current log filename
    * @return
    */
    string getFileName() { return getFileName(fileNumber); }

    LogStats getLogStats() { return s; }

    /**
     * Nonblocking call.
     *
     * \return true if the logger is started and ready to accept data.
     */
    bool isStarted() const { return started; }

    /**
     * Nonblocking call. Call this function to log a class.
     * \param t the class to be logged. This class has the following
     * requirements:
     * - it must be trivially_copyable, so no pointers or references inside
     *   the class, no stl containers, no virtual functions, no base classes.
     * - it must have a "void print(std::ostream& os) const" member function
     *   that prints all its data fields in text form (this is not used by the
     *   logger, but by the log decoder program)
     * \return whether the class has been logged
     */
    template <typename T>
    LogResult log(const T &t)
    {
        // static_assert(std::is_trivially_copyable<T>::value,"");
        return logImpl(typeid(t).name(), &t, sizeof(t));
    }

private:
    Logger();
    Logger(const Logger &) = delete;
    Logger &operator=(const Logger &) = delete;

    static void packThreadLauncher(void *argv);
    static void writeThreadLauncher(void *argv);
    static void statsThreadLauncher(void *argv);

    /**
     * Non-template dependente part of log
     * \param name class anem
     * \param data pointer to class data
     * \param size class size
     */
    LogResult logImpl(const char *name, const void *data, unsigned int size);

    /**
     * This thread packs logged data into buffers
     */
    void packThread();

    /**
     * This thread writes packed buffers to disk
     */
    void writeThread();

    /**
     * This thread prints stats
     */
    void statsThread();

    /**
     * Log logger stats using the logger itself
     */
    void logStats()
    {
        s.setTimestamp(miosix::getTick());
        log(s);
    }

    static const unsigned int filenameMaxRetry =
        100;                                         ///< Limit on new filename
    static const unsigned int maxRecordSize = 256;   ///< Limit on logged data
    static const unsigned int numRecords    = 1024;  ///< Size of record queues
    static const unsigned int bufferSize = 64 * 1024;  ///< Size of each buffer
    static const unsigned int numBuffers = 8;          ///< Number of buffers

    /**
     * A record is a single serialized logged class. Records are used to
     * make log() lock-free. Since each call to log() works on its independent
     * Record, calls to log do not need a global mutex which could block
     * threads calling log() concurrently
     */
    class Record
    {
    public:
        Record() : size(0) {}
        char data[maxRecordSize] = {};
        unsigned int size;
    };

    /**
     * A buffer is what is written on disk. It is filled by packing records.
     * The reason why we don't write records directly is that they are too
     * small to efficiently use disk bandwidth. SD cards are much faster when
     * data is written in large chunks.
     */
    class Buffer
    {
    public:
        Buffer() : size(0) {}
        char data[bufferSize] = {};
        unsigned int size;
    };

    int fileNumber = -1;

    miosix::Queue<Record *, numRecords> fullQueue;        ///< Full records
    miosix::Queue<Record *, numRecords> emptyQueue;       ///< Empty Records
    std::queue<Buffer *, std::list<Buffer *>> fullList;   ///< Full buffers
    std::queue<Buffer *, std::list<Buffer *>> emptyList;  ///< Empty buffers
    miosix::FastMutex mutex;  ///< To allow concurrent access to the queues
    miosix::ConditionVariable cond;  ///< To lock when buffers are all empty

    miosix::Thread *packT;   ///< Thread packing logged data
    miosix::Thread *writeT;  ///< Thread writing data to disk
    miosix::Thread *statsT;  ///< Thred printing stats

    volatile bool started = false;  ///< Logger is started and accepting data

    FILE *file;  ///< Log file
    LogStats s;  ///< Logger stats
};
