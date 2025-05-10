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
#include <sys/stat.h>
#include <sys/types.h>
#include <tscpp/buffer.h>

#include <chrono>
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

    // Find the proper log filename base on the current files on the disk
    string logName;
    string mappingName;
    for (fileNumber = 0; fileNumber < (int)maxFilenameNumber; fileNumber++)
    {
        // Check if the current file does not exists
        logName     = getLogName(fileNumber);
        mappingName = getMappingName(fileNumber);

        // TODO decide how to handle this part
        struct stat st;
        if (stat(logName.c_str(), &st) != 0)
            break;

        if (fileNumber == maxFilenameNumber - 1)
            TRACE("Too many log files, appending data to last\n");
    }

    logFile = fopen(logName.c_str(), "ab");  // b for binary
    if (logFile == NULL)
    {
        fileNumber = -1;
        TRACE("Error opening %s file\n", logName.c_str());
        return false;
    }

    mappingFile = fopen(mappingName.c_str(), "ab");  // b for binary
    if (mappingFile == NULL)
    {
        fileNumber = -1;
        TRACE("Error opening %s file\n", mappingName.c_str());
        return false;
    }

    setbuf(logFile, NULL);      // Disable buffering for the file stream
    setbuf(mappingFile, NULL);  // Disable buffering for the file stream

    // The boring part, start threads one by one and if they fail, undo.
    // Perhaps excessive defensive programming as thread creation failure is
    // highly unlikely (only if ram is full).

    packTh = Thread::create(packThreadLauncher, STACK_MIN_FOR_SKYWARD, 1, this,
                            Thread::JOINABLE);
    if (!packTh)
    {
        fclose(logFile);
        fclose(mappingFile);
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
        fclose(logFile);
        fclose(mappingFile);
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

    fullRecordsQueue.put(nullptr);         // Signal packThread to stop
    fullMappingRecordsQueue.put(nullptr);  // Signal packThread to stop

    packTh->join();
    writeTh->join();

    fclose(logFile);
    fclose(mappingFile);

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
    // Allocate the records
    for (unsigned int i = 0; i < numRecords; i++)
        emptyRecordsQueue.put(new Record);

    // Allocate buffers and put them in the empty list
    for (unsigned int i = 0; i < numBuffers; i++)
        emptyBufferList.push(new Buffer);
}

string Logger::getLogName(int logNumber)
{
    char filename[32];
    sprintf(filename, "/sd/log%02d.dat", logNumber);

    return string(filename);
}

string Logger::getMappingName(int logNumber)
{
    char filename[32];
    sprintf(filename, "/sd/mapping%02d.dat", logNumber);

    return string(filename);
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
                    return;
                }

                memcpy(buffer->data + buffer->size, record->data, record->size);
                buffer->size += record->size;
                emptyRecordsQueue.put(record);
            } while (bufferSize - buffer->size >= maxRecordSize);

            {
                Lock<FastMutex> l(mutex);
                // Put back full buffer
                fullBufferList.push(buffer);
                cond.broadcast();
                stats.buffersFilled++;
            }

            // packing for the mappings
            Buffer* mappingBuffer = nullptr;
            {
                Lock<FastMutex> l(mapMutex);
                // Get an empty buffer, wait if none is available
                while (emptyMappingBufferList.empty())
                    mapCond.wait(l);
                mappingBuffer = emptyMappingBufferList.front();
                emptyMappingBufferList.pop();
                mappingBuffer->size = 0;
            }
            do
            {
                Record* record = nullptr;
                fullMappingRecordsQueue.get(record);

                // When stop() is called, it pushes a nullptr signaling to stop
                if (record == nullptr)
                {
                    Lock<FastMutex> l(mapMutex);
                    fullMappingBufferList.push(mappingBuffer);  // Don't lose
                                                                // the buffer
                    fullMappingBufferList.push(nullptr);  // Signal writeThread
                                                          // to stop
                    mapCond.broadcast();
                    stats.buffersFilled++;
                    return;
                }

                memcpy(mappingBuffer->data + mappingBuffer->size, record->data,
                       record->size);
                mappingBuffer->size += record->size;
                emptyMappingRecordsQueue.put(record);
            } while (bufferSize - mappingBuffer->size >= maxRecordSize);

            {
                Lock<FastMutex> l(mapMutex);
                // Put back full buffer
                fullMappingBufferList.push(mappingBuffer);
                mapCond.broadcast();
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

            // writing for the samples
            Buffer* buffer = nullptr;
            {
                Lock<FastMutex> l(mutex);
                // Get a full buffer, wait if none is available
                while (fullBufferList.empty())
                    cond.wait(l);
                buffer = fullBufferList.front();
                fullBufferList.pop();
            }

            Buffer* mappingBuffer = nullptr;
            {
                Lock<FastMutex> l(mapMutex);
                // Get a full buffer, wait if none is available
                while (fullMappingBufferList.empty())
                    mapCond.wait(l);
                mappingBuffer = fullMappingBufferList.front();
                fullMappingBufferList.pop();
            }

            // When packThread stops, it pushes a nullptr signaling to
            // stop
            if (buffer == nullptr || mappingBuffer == nullptr)
                return;

            // Write data to disk
            using namespace std::chrono;
            auto start = system_clock::now();

            // write samples data
            size_t result = fwrite(buffer->data, 1, buffer->size, logFile);
            if (result != buffer->size)
            {
                // If this fails and your board uses SDRAM,
                // define and increase OVERRIDE_SD_CLOCK_DIVIDER_MAX
                stats.writesFailed++;
                stats.lastWriteError = ferror(logFile);
            }
            else
            {
                stats.buffersWritten++;
            }

            // write mapping data
            result = fwrite(mappingBuffer->data, 1, mappingBuffer->size,
                            mappingFile);
            if (result != mappingBuffer->size)
            {
                // If this fails and your board uses SDRAM,
                // define and increase OVERRIDE_SD_CLOCK_DIVIDER_MAX
                stats.writesFailed++;
                stats.lastWriteError = ferror(mappingFile);
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
                // Put back empty mapping buffer
                emptyMappingBufferList.push(mappingBuffer);
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
