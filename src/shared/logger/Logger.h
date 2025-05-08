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

#include <atomic>
#include <cstdio>
#include <list>
#include <queue>
#include <string>
#include <type_traits>
#include <userde.hpp>

#include "LoggerStats.h"

namespace Boardcore
{

template <typename T, typename = void>
struct TypeID
{
    static constexpr char VALUE = '?';
};

#define TYPEID_STRUCT(type, character)           \
    template <>                                  \
    struct TypeID<type>                          \
    {                                            \
        static constexpr char VALUE = character; \
    };

TYPEID_STRUCT(bool, 'b')
TYPEID_STRUCT(char, 'c')
TYPEID_STRUCT(int8_t, 'h')
TYPEID_STRUCT(uint8_t, 'H')
TYPEID_STRUCT(int16_t, 'i')
TYPEID_STRUCT(uint16_t, 'I')
TYPEID_STRUCT(int32_t, 'j')
TYPEID_STRUCT(uint32_t, 'J')
TYPEID_STRUCT(int64_t, 'l')
TYPEID_STRUCT(uint64_t, 'L')
TYPEID_STRUCT(float, 'f')
TYPEID_STRUCT(double, 'd')

template <typename T>
struct TypeID<T, std::enable_if_t<std::is_enum<T>::value>>
{
    static constexpr char VALUE = TypeID<std::underlying_type_t<T>>::VALUE;
};

template <typename T>
struct TypePrinter
{
    static constexpr void print(std::string& mappingString,
                                uint8_t* numberOfTypes, const char* name)
    {
        mappingString += std::string(name);
        mappingString += '\0';
        mappingString += TypeID<T>::VALUE;
        mappingString += '\0';

        (*numberOfTypes) += 1;
    }
};

template <typename T, size_t I>
struct TypePrinter<T[I]>
{
    static constexpr void print(std::string& mappingString,
                                uint8_t* numberOfTypes, const char* name)
    {
        for (unsigned int i = 0; i < I; i++)
        {
            mappingString += std::string(name) + "[" + std::to_string(i) + "]";
            mappingString += '\0';
            mappingString += TypeID<T>::VALUE;
            mappingString += '\0';

            (*numberOfTypes) += 1;
        }
    }
};

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

    std::string getCurrentMappingName();

    LoggerStats getStats();

    void resetStats();

    bool isStarted() const;

    /**
     * @brief Call this function to log a class.
     *
     * Nonblocking call.
     *
     *
     * \param T The class to be logged. This class has the following
     * requirements:
     * - the class must have a reflect method that is compliant
     *   with the library socrate (https://git.skywarder.eu/avn/swd/socrate)
     *
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

    static constexpr unsigned int numMappingRecords =
        32;  ///< Size of mapping record queues
    static constexpr unsigned int numMappingBuffers =
        4;  ///< Number of mapping buffers
#else
    static constexpr unsigned int maxRecordSize = 512;  ///< Limit on data
    static constexpr unsigned int numRecords = 64;  ///< Size of record queues
    static constexpr unsigned int numBuffers = 8;   ///< Number of buffers
    static constexpr unsigned int bufferSize = 4 * 1024;  ///< Size of buffers

    static constexpr unsigned int numMappingRecords =
        16;  ///< Size of mapping record queues
    static constexpr unsigned int numMappingBuffers =
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

    miosix::Queue<Record*, numRecords> fullMappingRecordsQueue;
    miosix::Queue<Record*, numRecords> emptyMappingRecordsQueue;
    std::queue<Buffer*, std::list<Buffer*>> fullMappingBufferList;
    std::queue<Buffer*, std::list<Buffer*>> emptyMappingBufferList;

    miosix::FastMutex mutex;  ///< To allow concurrent access to the queues.
    miosix::ConditionVariable cond;  ///< To lock when buffers are all empty.

    miosix::Thread* packTh  = nullptr;  ///< Thread packing logged data.
    miosix::Thread* writeTh = nullptr;  ///< Thread writing data to disk.

    std::atomic<bool> started{
        false};  ///< Logger is started and accepting data.

    ///< signal writing thread to stop
    std::atomic<bool> stopWritingThread{false};

    FILE* logFile     = nullptr;  ///< Log file.
    FILE* mappingFile = nullptr;  ///< Mapping file.
    LoggerStats stats;            ///< Logger stats.

    static_assert(
        sizeof(int32_t) == sizeof(int),
        "Int type should be 32 bits in size, if that is not the case "
        "consider updating the reinterpret_cast in the atomicAdd calls");
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
        miosix::FastInterruptDisableLock dLock;
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
        miosix::atomicAdd(reinterpret_cast<int*>(&stats.tooLargeSamples), 1);
        TRACE("The current record size is not enough to store %s\n",
              T::reflect().type_name());
        return LoggerResult::TooLarge;
    }

    record->size = socrate::userde::Serde<T>::size() +
                   strlen(T::reflect().type_name()) + 1;

    // Move the record to the full queue, where the pack thread will read and
    // store it in a buffer
    fullRecordsQueue.put(record);

    miosix::atomicAdd(reinterpret_cast<int*>(&stats.queuedSamples), 1);

    return LoggerResult::Queued;
}

// This function might not work with non standard types but that's a future me
// problem

template <typename T>
void Logger::mapType(T& t)
{
    Record* record = nullptr;
    // Retrieve a record from the empty queue, if available
    {
        // We disable interrupts because IRQget() is nonblocking, unlike get()
        miosix::FastInterruptDisableLock dLock;
        if (emptyMappingRecordsQueue.IRQget(record) == false)
        {
            stats.droppedSamples++;
            return;
        }
    }

    std::string mappingString;         // This is the complete mapping string
    std::string partialMappingString;  // This string only conains the type
                                       // names and their typeIDs
    uint8_t numberOfTypes =
        0;  // This is used to get the number of fields in the type
            // (we want to count all of the elements of an array)

    mappingString += T::reflect().type_name();
    mappingString += '\0';

    T::reflect().for_each_field_type(
        [&](const char* name, auto type)
        {
            using Type = typename decltype(type)::Type;
            TypePrinter<Type>::print(partialMappingString, &numberOfTypes,
                                     name);
        });

    // now that we have all of the information we need we can merge the mapping
    // strings
    mappingString += numberOfTypes;
    mappingString += '\0';
    mappingString += partialMappingString;

    // check if the mapping size is too large for the record, if it's too small
    // move the record to the empty queue
    if (mappingString.size() > maxRecordSize)
    {
        emptyMappingRecordsQueue.put(record);
        TRACE(
            "The current record size is not enough to store the mapping of "
            "%s\n",
            T::reflect().type_name());
        return;
    }

    // If the record is big enough, copy the mapping string in the record
    memcpy(record->data, mappingString.c_str(), mappingString.size());

    // Move the record to the full queue, where the pack thread will read and
    // store it in a buffer
    record->size = mappingString.size();
    fullMappingRecordsQueue.put(record);

    miosix::atomicAdd(reinterpret_cast<int*>(&stats.queuedMappings), 1);
}

}  // namespace Boardcore

