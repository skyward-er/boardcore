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

#include <Singleton.h>
#include <miosix.h>

#include <cstdio>
#include <list>
#include <queue>
#include <string>
#include <type_traits>

#include "LogStats.h"

using std::string;

namespace Boardcore
{

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
class Logger : public Singleton<Logger>
{
    friend class Singleton<Logger>;

public:
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
        static_assert(
            std::is_trivially_copyable<T>::value,
            "A type T must be trivially copyable in order to be logged!");

        return logImpl(typeid(t).name(), &t, sizeof(t));
    }

private:
    Logger();

    static void packThreadLauncher(void *argv);
    static void writeThreadLauncher(void *argv);

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
     * Log logger stats using the logger itself
     */
    void logStats()
    {
        s.setTimestamp(miosix::getTick());
        log(s);
    }

    static const unsigned int filenameMaxRetry =
        100;                                        ///< Limit on new filename
    static const unsigned int maxRecordSize = 512;  ///< Limit on logged data
    static const unsigned int numRecords    = 512;  ///< Size of record queues
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

    miosix::Thread *packT  = nullptr;  ///< Thread packing logged data
    miosix::Thread *writeT = nullptr;  ///< Thread writing data to disk
    // miosix::Thread *statsT;  ///< Thred printing stats

    volatile bool started = false;  ///< Logger is started and accepting data

    FILE *file = nullptr;  ///< Log file
    LogStats s;            ///< Logger stats
};

}  // namespace Boardcore
