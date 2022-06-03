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
#include <stdint.h>

#include <cstdio>
#include <list>
#include <queue>
#include <string>
#include <type_traits>

#include "LoggerStats.h"

namespace Boardcore
{

/**
 * @brief Possible outcomes of Logger::log().
 */
enum class LoggerResult
{
    Queued,   ///< Data has been accepted by the logger and will be written.
    Dropped,  ///< Buffers are currently full, data will not be written. Sorry.
    Ignored,  ///< Logger is currently stopped, data will not be written.
    TooLarge  ///< Data is too large to be logged. Increase maxRecordSize.
};

/**
 * @brief Buffered logger. Needs to be started before it can be used.
 */
class Logger : public Singleton<Logger>
{
    friend class Singleton<Logger>;

public:
    /**
     * @brief Call this function to start the logger.
     *
     * The function tryies to start the logger. It first opens the log file and
     * then create the pack and write threads. If it fails on one of this
     * operation, the logger is not started.
     *
     * Use getCurrentLogNumber to retrieve the log file number.
     *
     * Blocking call. May take a long time.
     *
     * \return true if the logger was started correctly.
     */
    bool start();

    /**
     * @brief Call this function to stop the logger.
     *
     * When this function returns, all log buffers have been flushed to disk,
     * and it is safe to power down the board without losing log data or
     * corrupting the filesystem.
     *
     * Blocking call. May take a very long time (seconds).
     */
    void stop();

    /**
     * @brief Tests if the Logger can write to the SD card by opening a file.
     */
    static bool testSDCard();

    int getCurrentLogNumber();

    std::string getCurrentFileName();

    LoggerStats getLoggerStats();

    bool isStarted() const;

    /**
     * @brief Call this function to log a class.
     *
     * Nonblocking call.
     *
     * Refer to the TSCPP repository to know more about how the data is stored.
     * https://git.skywarder.eu/scs/third-party/tscpp
     *
     * \param T The class to be logged. This class has the following
     * requirements:
     * - it must be trivially_copyable, so no pointers or references inside
     *   the class, no stl containers, no virtual functions, no base classes.
     * - it must have a "void print(std::ostream& os) const" member function
     *   that prints all its data fields in text form (this is not used by the
     *   logger, but by the log decoder program)
     * \return Whether the class has been logged.
     */
    template <typename T>
    LoggerResult log(const T &t);

    /**
     * @brief Log logger stats using the logger itself.
     */
    void logStats();

private:
    Logger();

    static std::string getFileName(int logNumber);

    static void packThreadLauncher(void *argv);

    static void writeThreadLauncher(void *argv);

    /**
     * This thread packs logged data into buffers
     */
    void packThread();

    /**
     * This thread writes packed buffers to disk
     */
    void writeThread();

    /**
     * @brief Implementation of the log function non-template dependent.
     *
     * \param name Class name.
     * \param data Pointer to class data.
     * \param size Class size.
     */
    LoggerResult logImpl(const char *name, const void *data, unsigned int size);

    static constexpr unsigned int maxFilenameNumber = 100;  ///< Limit on files
    static constexpr unsigned int maxRecordSize     = 512;  ///< Limit on data
    static constexpr unsigned int numRecords = 512;  ///< Size of record queues
    static constexpr unsigned int numBuffers = 8;    ///< Number of buffers
    static constexpr unsigned int bufferSize = 64 * 1024;  ///< Size of buffers

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

    miosix::Queue<Record *, numRecords> fullRecordsQueue;
    miosix::Queue<Record *, numRecords> emptyRecordsQueue;
    std::queue<Buffer *, std::list<Buffer *>> fullBufferList;
    std::queue<Buffer *, std::list<Buffer *>> emptyBufferList;
    miosix::FastMutex mutex;  ///< To allow concurrent access to the queues.
    miosix::ConditionVariable cond;  ///< To lock when buffers are all empty.

    miosix::Thread *packTh  = nullptr;  ///< Thread packing logged data.
    miosix::Thread *writeTh = nullptr;  ///< Thread writing data to disk.

    volatile bool started = false;  ///< Logger is started and accepting data.

    FILE *file = nullptr;  ///< Log file.
    LoggerStats stats;     ///< Logger stats.
};

template <typename T>
LoggerResult Logger::log(const T &t)
{
    static_assert(
        std::is_trivially_copyable<T>::value,
        "The type T must be trivially copyable in order to be logged!");

    return logImpl(typeid(t).name(), &t, sizeof(t));
}

}  // namespace Boardcore
