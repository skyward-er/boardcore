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

#include "Logger.h"

#include <diagnostic/SkywardStack.h>
#include <diagnostic/StackLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <errno.h>
#include <fcntl.h>
#include <fmt/format.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <chrono>
#include <cstring>
#include <fstream>
#include <stdexcept>

using namespace std;
using namespace miosix;

namespace Boardcore
{

bool Logger::start()
{
    if (started)
        return true;

    // Find the next log filename based on the current files on the disk
    int logNumber = findNextLogNumber();
    if (logNumber < 0)
    {
        TRACE("Too many log files, aborting\n");
        return false;
    }

    string logName = getLogName(logNumber);

    file = fopen(logName.c_str(), "ab");  // b for binary
    if (file == NULL)
    {
        fileNumber = -1;
        TRACE("Error opening %s file\n", logName.c_str());
        return false;
    }
    else
    {
        fileNumber = logNumber;
    }

    setbuf(file, NULL);  // Disable buffering for the file stream

    // The boring part, start threads one by one and if they fail, undo.
    // Perhaps excessive defensive programming as thread creation failure is
    // highly unlikely (only if ram is full).

    packTh = Thread::create(packThreadLauncher, STACK_MIN_FOR_SKYWARD, 1, this,
                            Thread::JOINABLE);
    if (!packTh)
    {
        fclose(file);
        TRACE("Error creating pack thread\n");
        return false;
    }

    writeTh = Thread::create(writeThreadLauncher, STACK_MIN_FOR_SKYWARD, 1,
                             this, Thread::JOINABLE);
    if (!writeTh)
    {
        fullRecordsQueue.put(nullptr);  // Signal packThread to stop
        packTh->join();
        // packThread has pushed a buffer and a nullptr to writeThread, remove
        // it
        while (fullBufferList.front() != nullptr)
        {
            emptyBufferList.push(fullBufferList.front());
            fullBufferList.pop();
        }
        fullBufferList.pop();  // Remove nullptr
        fclose(file);
        TRACE("Error creating write thread\n");
        return false;
    }

    started = true;

    return true;
}

void Logger::stop()
{
    if (started == false)
        return;
    logStats();

    started = false;

    fullRecordsQueue.put(nullptr);  // Signal packThread to stop

    packTh->join();
    writeTh->join();

    fclose(file);

    stats = {};

    fileNumber = -1;  // Reset the fileNumber to an invalid value
}

bool Logger::testSDCard()
{
    bool result = ofstream("/sd/test").good();
    std::remove("/sd/test");
    return result;
}

int Logger::getCurrentLogNumber() { return fileNumber; }

string Logger::getCurrentFileName() { return getLogName(fileNumber); }

LoggerStats Logger::getStats()
{
    stats.timestamp = TimestampTimer::getTimestamp();
    stats.logNumber = fileNumber;
    return stats;
}

void Logger::resetStats()
{
    // Keep some of the statistics persistent
    int buffersWritten = stats.buffersWritten;
    int writesFailed   = stats.writesFailed;

    // Reset
    stats = {};

    // Put back
    stats.buffersWritten = buffersWritten;
    stats.writesFailed   = writesFailed;
}

bool Logger::isStarted() const { return started; }

void Logger::logStats()
{
    log(getStats());
    resetStats();
}

Logger::Logger()
{
    // Allocate the records for the log
    for (unsigned int i = 0; i < numRecords; i++)
        emptyRecordsQueue.put(new Record);

    // Allocate the records for the mappings
    for (unsigned int i = 0; i < numMappings; i++)
        emptyMappingsQueue.put(new MappingRecord);

    // Allocate buffers for the log and put them in the empty list
    for (unsigned int i = 0; i < numBuffers; i++)
        emptyBufferList.push(new Buffer);
}

int Logger::findNextLogNumber()
{
    int low  = 1;
    int high = maxFilenameNumber;

    while (low <= high)
    {
        int mid             = low + (high - low) / 2;
        std::string logName = getLogName(mid);

        struct stat st;
        if (stat(logName.c_str(), &st) == 0)
        {
            // File exists, so the next available number must be higher
            low = mid + 1;
        }
        else
        {
            // File does not exist, so this could be the one, or it could be a
            // lower number
            high = mid - 1;
        }
    }

    // After the loop `low` holds the first number for which a log was not found
    if (low > maxFilenameNumber)
    {
        // If we reached the maximum number, return -1 to indicate no available
        // log numbers
        return -1;
    }

    return low;
}

string Logger::getLogName(int logNumber)
{
    return fmt::format("/sd/log{:02d}.dat", logNumber);
}

void Logger::packThreadLauncher(void* argv)
{
    reinterpret_cast<Logger*>(argv)->packThread();
}

void Logger::writeThreadLauncher(void* argv)
{
    reinterpret_cast<Logger*>(argv)->writeThread();
}

void Logger::packThread()
{
    /*
     * The first implementation of this class had the log() function write
     * directly the serialized data to the buffers. So, no Records nor
     * packThread existed. However, to be able to call log() concurrently
     * without messing up the buffer, a mutex was needed. Thus, if many
     * threads call log(), contention on that mutex would occur, serializing
     * accesses and slowing down the (potentially real-time) callers. For this
     * reason Records and the pack thread were added.
     * Now each log() works independently on its own Record, and log() accesses
     * can proceed in parallel.
     */

    try
    {
        for (;;)
        {
            StackLogger::getInstance().updateStack(THID_LOGGER_PACK);

            // packing for the samples
            Buffer* buffer = nullptr;
            {
                Lock<FastMutex> l(mutex);
                // Get an empty buffer, wait if none is available
                while (emptyBufferList.empty())
                    cond.wait(l);
                buffer = emptyBufferList.front();
                emptyBufferList.pop();
                buffer->size = 0;
            }

            do
            {
                Record* record = nullptr;
                fullRecordsQueue.get(record);

                // When stop() is called, it pushes a nullptr signaling to stop
                if (record == nullptr)
                {
                    Lock<FastMutex> l(mutex);
                    fullBufferList.push(buffer);   // Don't lose the buffer
                    fullBufferList.push(nullptr);  // Signal writeThread to stop

                    cond.broadcast();
                    stats.buffersFilled++;
                    return;  // Stop the thread
                }

                // If the record has a mapping, we need to write it before
                // writing the data
                if (record->mapping)
                {
                    auto* mapping = record->mapping;
                    memcpy(buffer->data + buffer->size, mapping->data,
                           mapping->size);
                    buffer->size += mapping->size;
                    emptyMappingsQueue.put(mapping);
                }

                memcpy(buffer->data + buffer->size, record->data, record->size);
                buffer->size += record->size;
                emptyRecordsQueue.put(record);
            } while (bufferSize - buffer->size >=
                     maxRecordSize + maxMappingSize);

            {
                Lock<FastMutex> l(mutex);
                // Put back full buffer
                fullBufferList.push(buffer);
                cond.broadcast();
                stats.buffersFilled++;
            }
        }
    }
    catch (exception& e)
    {
        TRACE("Error: packThread failed due to an exception: %s\n", e.what());
    }
}

void Logger::writeThread()
{
    try
    {
        for (;;)
        {
            StackLogger::getInstance().updateStack(THID_LOGGER_WRITE);

            Buffer* buffer = nullptr;
            {
                Lock<FastMutex> l(mutex);
                // Get a full buffer, wait if none is available
                while (fullBufferList.empty())
                    cond.wait(l);
                buffer = fullBufferList.front();
                fullBufferList.pop();
            }

            // When packThread stops, it pushes a nullptr signaling to
            // stop
            if (buffer == nullptr)
                return;

            // Write data to disk
            using namespace std::chrono;
            auto start = system_clock::now();

            size_t result = fwrite(buffer->data, 1, buffer->size, file);
            if (result != buffer->size)
            {
                // If this fails and your board uses SDRAM,
                // define and increase OVERRIDE_SD_CLOCK_DIVIDER_MAX
                stats.writesFailed++;
                stats.lastWriteError = ferror(file);
            }
            else
            {
                stats.buffersWritten++;
            }

            auto interval = system_clock::now() - start;
            stats.averageWriteTime =
                duration_cast<milliseconds>(interval).count();
            stats.maxWriteTime =
                max(stats.maxWriteTime, stats.averageWriteTime);

            {
                Lock<FastMutex> l(mutex);

                // Put back empty buffer
                emptyBufferList.push(buffer);
                cond.broadcast();
            }
        }
    }
    catch (exception& e)
    {
        TRACE("Error: writeThread failed due to an exception: %s\n", e.what());
    }
}

}  // namespace Boardcore
