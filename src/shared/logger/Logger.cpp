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
#include <interfaces/atomic_ops.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <tscpp/buffer.h>
#include <utils/Debug.h>

#include <stdexcept>

using namespace std;
using namespace miosix;

namespace Boardcore
{

int Logger::start()
{
    if (started)
        return fileNumber;

    // Find the proper log filename base on the current files on the disk
    string filename;
    for (fileNumber = 0; fileNumber < (int)maxFilenameNumber; fileNumber++)
    {
        // Check if the current file does not exists
        filename = getFileName(fileNumber);
        struct stat st;
        if (stat(filename.c_str(), &st) != 0)
        {
            break;
        }

        if (fileNumber == maxFilenameNumber - 1)
            TRACE("Too many log files, appending data to last\n");
    }

    file = fopen(filename.c_str(), "ab");  // b for binary
    if (file == NULL)
    {
        fileNumber = -1;
        TRACE("Error opening %s file\n", filename.c_str());
        throw runtime_error("Error opening log file");
    }
    setbuf(file, NULL);  // Disable buffering for the file stream

    // The boring part, start threads one by one and if they fail, undo
    // Perhaps excessive defensive programming as thread creation failure is
    // highly unlikely (only if ram is full)

    packTh = Thread::create(packThreadLauncher, skywardStack(16 * 1024), 1,
                            this, Thread::JOINABLE);
    if (!packTh)
    {
        fclose(file);
        TRACE("Error creating pack thread\n");
        throw runtime_error("Error creating pack thread");
    }

    writeTh = Thread::create(writeThreadLauncher, skywardStack(16 * 1024), 1,
                             this, Thread::JOINABLE);
    if (!writeTh)
    {
        fullQueue.put(nullptr);  // Signal packThread to stop
        packTh->join();
        // packThread has pushed a buffer and a nullptr to writeThread, remove
        // it
        while (fullList.front() != nullptr)
        {
            emptyList.push(fullList.front());
            fullList.pop();
        }
        fullList.pop();  // Remove nullptr
        fclose(file);
        TRACE("Error creating write thread\n");
        throw runtime_error("Error creating write thread");
    }

    started         = true;
    stats.logNumber = fileNumber;

    return fileNumber;
}

void Logger::stop()
{
    if (started == false)
        return;
    logStats();

    started = false;

    fullQueue.put(nullptr);  // Signal packThread to stop

    packTh->join();
    writeTh->join();

    fclose(file);
}

int Logger::getLogNumber() { return fileNumber; }

string Logger::getFileName(int log_number)
{
    char filename[32];
    sprintf(filename, "/sd/log%02d.dat", log_number);

    return string(filename);
}

string Logger::getCurrentFileName() { return getFileName(fileNumber); }

LoggerStats Logger::getLoggerStats() { return stats; }

bool Logger::isStarted() const { return started; }

Logger::Logger()
{
    // Allocate the records
    for (unsigned int i = 0; i < numRecords; i++)
        emptyQueue.put(new Record);

    // Allocate buffers and put them in the empty list
    for (unsigned int i = 0; i < numBuffers; i++)
        emptyList.push(new Buffer);
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

            Buffer* buffer = nullptr;
            {
                Lock<FastMutex> l(mutex);
                // Get an empty buffer, wait if none is available
                while (emptyList.empty())
                    cond.wait(l);
                buffer = emptyList.front();
                emptyList.pop();
                buffer->size = 0;
            }

            do
            {
                Record* record = nullptr;
                fullQueue.get(record);

                // When stop() is called, it pushes a nullptr signaling to stop
                if (record == nullptr)
                {
                    Lock<FastMutex> l(mutex);
                    fullList.push(buffer);   // Don't lose the buffer
                    fullList.push(nullptr);  // Signal writeThread to stop
                    cond.broadcast();
                    stats.buffersFilled++;
                    return;
                }

                memcpy(buffer->data + buffer->size, record->data, record->size);
                buffer->size += record->size;
                emptyQueue.put(record);
            } while (bufferSize - buffer->size >= maxRecordSize);

            {
                Lock<FastMutex> l(mutex);
                // Put back full buffer
                fullList.push(buffer);
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
                while (fullList.empty())
                    cond.wait(l);
                buffer = fullList.front();
                fullList.pop();
            }

            // When packThread stops, it pushes a nullptr signaling to stop
            if (buffer == nullptr)
                return;

            // Write data to disk
            Timer timer;
            timer.start();

            size_t result = fwrite(buffer->data, 1, buffer->size, file);
            if (result != buffer->size)
            {
                // If this fails and your board uses SDRAM,
                // define and increase OVERRIDE_SD_CLOCK_DIVIDER_MAX
                stats.writesFailed++;
                stats.lastWriteError = ferror(file);
            }
            else
                stats.buffersWritten++;

            // green_led::low();
            timer.stop();
            stats.writeTime    = timer.interval();
            stats.maxWriteTime = max(stats.maxWriteTime, stats.writeTime);

            {
                Lock<FastMutex> l(mutex);

                // Put back empty buffer
                emptyList.push(buffer);
                cond.broadcast();
            }
        }
    }
    catch (exception& e)
    {
        TRACE("Error: writeThread failed due to an exception: %s\n", e.what());
    }
}

LoggerResult Logger::logImpl(const char* name, const void* data,
                             unsigned int size)
{
    if (started == false)
    {
        TRACE("Attempting to log %s but the Logger is not started!\n", name);
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
        if (emptyQueue.IRQget(record) == false)
        {
            stats.droppedSamples++;
            return LoggerResult::Dropped;
        }
    }

    // Copy the data in the record
    int result =
        tscpp::serializeImpl(record->data, maxRecordSize, name, data, size);

    // If the record is too small, move the record in the empty queue and error
    if (result == tscpp::BufferTooSmall)
    {
        emptyQueue.put(record);
        atomicAdd(&stats.tooLargeSamples, 1);
        TRACE("The current record size is not enough to store %s\n", name);
        return LoggerResult::TooLarge;
    }

    record->size = result;

    // Move the record to the full queue, where the pack thread will read and
    // store it in a buffer
    fullQueue.put(record);

    atomicAdd(&stats.queuedSamples, 1);

    return LoggerResult::Queued;
}

void Logger::logStats()
{
    stats.timestamp = TimestampTimer::getTimestamp();
    log(stats);
}

}  // namespace Boardcore
