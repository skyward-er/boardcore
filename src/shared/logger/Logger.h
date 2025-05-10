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
#include <interfaces/atomic_ops.h>
#include <miosix.h>
#include <stdint.h>
#include <utils/Debug.h>

#include <cstdio>
#include <list>
#include <queue>
#include <string>
#include <type_traits>
#include <userde.hpp>

#include "LoggerStats.h"

using namespace miosix;

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
     * Tries to start the logger, first opens the log file and then create the
     * pack and write threads. If it fails on one of this operation, the logger
     * is not started.
     *
     * Use getCurrentLogNumber to retrieve the log file number.
     *
     * Blocking call. May take a long time.
     *
     * \return True if the logger was started correctly.
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

    LoggerStats getStats();

    void resetStats();

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
    LoggerResult log(const T& t);

    /**
     * @brief Log logger stats using the logger itself.
     *
     * The stats are reset after being logged.
     */
    void logStats();

    /**
     * @brief Returns the Max Filename number
     */
    static constexpr unsigned int getMaxFilenameNumber()
    {
        return maxFilenameNumber;
    }

private:
    Logger();

    static std::string getLogName(int logNumber);

    static std::string getMappingName(int logNumber);

    static void packThreadLauncher(void* argv);

    static void writeThreadLauncher(void* argv);

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
    template <typename T>
    LoggerResult logImpl(T& t, unsigned int size);

    /**
     * @brief Maps the type T to a mapping file.
     *
     * This function is used to map the type T to a mapping file. This file is
     * used by the log decoder.
     *
     * \param t The class to be mapped.
     */
    template <typename T>
    void mapType(T& t);

    static constexpr unsigned int maxFilenameNumber =
        10000;  ///< Limit on files
#ifndef _ARCH_CORTEXM3_STM32F2
    static constexpr unsigned int maxRecordSize = 512;  ///< Limit on data
    static constexpr unsigned int numRecords = 512;  ///< Size of record queues
    static constexpr unsigned int numBuffers = 8;    ///< Number of buffers
    static constexpr unsigned int bufferSize = 64 * 1024;  ///< Size of buffers

    static constexpr unsigned int numMapRecords =
        32;  ///< Size of mapping record queues
    static constexpr unsigned int numMapBuffers =
        4;  ///< Number of mapping buffers
#else
    static constexpr unsigned int maxRecordSize = 512;  ///< Limit on data
    static constexpr unsigned int numRecords = 64;  ///< Size of record queues
    static constexpr unsigned int numBuffers = 8;   ///< Number of buffers
    static constexpr unsigned int bufferSize = 4 * 1024;  ///< Size of buffers

    static constexpr unsigned int numMapRecords =
        16;  ///< Size of mapping record queues
    static constexpr unsigned int numMapBuffers =
        4;  ///< Number of mapping buffers

#endif

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
        uint8_t data[maxRecordSize] = {};
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
        uint8_t data[bufferSize] = {};
        unsigned int size;
    };

    int fileNumber = -1;

    miosix::Queue<Record*, numRecords> fullRecordsQueue;
    miosix::Queue<Record*, numRecords> emptyRecordsQueue;
    std::queue<Buffer*, std::list<Buffer*>> fullBufferList;
    std::queue<Buffer*, std::list<Buffer*>> emptyBufferList;
    miosix::FastMutex mutex;  ///< To allow concurrent access to the queues.
    miosix::ConditionVariable cond;  ///< To lock when buffers are all empty.

    miosix::Queue<Record*, numRecords> fullMappingRecordsQueue;
    miosix::Queue<Record*, numRecords> emptyMappingRecordsQueue;
    std::queue<Buffer*, std::list<Buffer*>> fullMappingBufferList;
    std::queue<Buffer*, std::list<Buffer*>> emptyMappingBufferList;
    miosix::FastMutex
        mapMutex;  ///< To allow concurrent access to the mapping queues.
    miosix::ConditionVariable
        mapCond;  ///< To lock when mapping buffers are all empty.

    miosix::Thread* packTh  = nullptr;  ///< Thread packing logged data.
    miosix::Thread* writeTh = nullptr;  ///< Thread writing data to disk.

    volatile bool started = false;  ///< Logger is started and accepting data.

    FILE* logFile     = nullptr;  ///< Log file.
    FILE* mappingFile = nullptr;  ///< Mapping file.
    LoggerStats stats;            ///< Logger stats.
};

template <typename T>
LoggerResult Logger::log(const T& t)
{
    static bool isMapped = false;

    if (!isMapped)
    {
        mapType(t);
        isMapped = true;
    }

    return logImpl(t, sizeof(t));
}

template <typename T>
LoggerResult Logger::logImpl(T& t, unsigned int size)
{
    T mutableCopy = t;
    if (started == false)
    {
        stats.droppedSamples++;

        // Signal that we are trying to write to a closed log
        stats.lastWriteError = -1;

        return LoggerResult::Ignored;
    }

    Record* record = nullptr;

    // Retrieve a record from the empty queue, if available
    {
        // We disable interrupts because IRQget() is nonblocking, unlike get()
        FastInterruptDisableLock dLock;
        if (emptyRecordsQueue.IRQget(record) == false)
        {
            stats.droppedSamples++;
            return LoggerResult::Dropped;
        }
    }

    // Copy the data in the record
    auto result = socrate::userde::serialize_with_name<T>(
        mutableCopy, record->data, maxRecordSize);

    // If the record is too small, move the record in the empty queue and
    // error
    if (result == socrate::userde::Error::BufferTooSmall)
    {
        emptyRecordsQueue.put(record);
        atomicAdd(&stats.tooLargeSamples, 1);
        TRACE("The current record size is not enough to store %s\n",
              t.type_name());
        return LoggerResult::TooLarge;
    }

    // Move the record to the full queue, where the pack thread will read and
    // store it in a buffer
    fullRecordsQueue.put(record);

    atomicAdd(&stats.queuedSamples, 1);

    return LoggerResult::Queued;
}

template <typename T>
void Logger::mapType(T& t)
{
    Record* record = nullptr;
    // Retrieve a record from the empty queue, if available
    {
        // We disable interrupts because IRQget() is nonblocking, unlike get()
        FastInterruptDisableLock dLock;
        if (emptyMappingRecordsQueue.IRQget(record) == false)
        {
            stats.droppedSamples++;
            return;
        }
    }
    // calculate the size of the mapping
    std::string typeName(T::reflect().type_name());

    size_t mappingSize;
    mappingSize = typeName.size() + 1;  // name of the type + null terminator
    mappingSize += 2;                   // field count + null terminator
    T::reflect().for_each_field(
        t,
        [&](const char* _name, auto& value)
        {
            std::string fieldName(_name);
            std::string type(typeid(value).name());
            mappingSize +=
                fieldName.size() + 1;        // field name + null terminator
            mappingSize += type.size() + 1;  // field type + null terminator
        });

    // check if the mapping size is too large for the record, if it's too small
    // move the record to the empty queue
    if (mappingSize > maxRecordSize)
    {
        emptyMappingRecordsQueue.put(record);
        TRACE(
            "The current record size is not enough to store the mapping of "
            "%s\n",
            t.type_name());
        return;
    }

    // Copy the data in the record
    memcpy(record->data, typeName.c_str(), typeName.size() + 1);

    size_t offset = 0;

    // Write the type name to the record
    memcpy(record->data + offset, typeName.c_str(), typeName.size() + 1);
    offset += typeName.size() + 1;

    // Write the field count to the record
    auto fieldCount = T::reflect().field_count();
    memcpy(record->data + offset, &fieldCount, 2);
    offset += 2;

    T::reflect().for_each_field(
        t,
        [&](const char* _name, auto& value)
        {
            std::string fieldName(_name);
            std::string type(typeid(value).name());

            // Write the field name to the record
            memcpy(record->data + offset, fieldName.c_str(),
                   fieldName.size() + 1);
            offset += fieldName.size() + 1;
            TRACE("field name: %s\n", fieldName.c_str());

            // Write the field type to the record
            memcpy(record->data + offset, type.c_str(), type.size() + 1);
            offset += type.size() + 1;

            TRACE("field type: %s\n", type.c_str());
        });

    // Move the record to the full queue, where the pack thread will read and
    // store it in a buffer
    fullMappingRecordsQueue.put(record);

    atomicAdd(&stats.queuedMappings, 1);
}

}  // namespace Boardcore
