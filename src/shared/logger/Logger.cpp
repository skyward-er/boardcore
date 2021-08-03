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

#include <errno.h>
#include <fcntl.h>
#include <interfaces/atomic_ops.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <tscpp/buffer.h>

#include <stdexcept>

#include "Debug.h"
#include "diagnostic/SkywardStack.h"
#include "diagnostic/StackLogger.h"

using namespace std;
using namespace miosix;
using namespace tscpp;

//
// class Logger
//

// typedef Gpio<GPIOG_BASE, 13> green_led;  // STM32F429ZI green led

Logger& Logger::instance()
{
    static Logger logger;
    return logger;
}

int Logger::start()
{
    if (started)
        return fileNumber;

    string filename;
    for (fileNumber = 0; fileNumber < (int)filenameMaxRetry; fileNumber++)
    {
        filename = getFileName(fileNumber);
        struct stat st;
        if (stat(filename.c_str(), &st) != 0)
        {
            break;
        }
        // File exists
        if (fileNumber == filenameMaxRetry - 1)
            puts("Too many files, appending to last");
    }

    file = fopen(filename.c_str(), "ab");
    if (file == NULL)
    {
        fileNumber = -1;
        throw runtime_error("Error opening log file");
    }
    setbuf(file, NULL);

    // The boring part, start threads one by one and if they fail, undo
    // Perhaps excessive defensive programming as thread creation failure is
    // highly unlikely (only if ram is full)

    packT = Thread::create(packThreadLauncher, skywardStack(16 * 1024), 1, this,
                           Thread::JOINABLE);
    if (!packT)
    {
        fclose(file);
        throw runtime_error("Error creating pack thread");
    }

    writeT = Thread::create(writeThreadLauncher, skywardStack(16 * 1024), 1,
                            this, Thread::JOINABLE);
    if (!writeT)
    {
        fullQueue.put(nullptr);  // Signal packThread to stop
        packT->join();
        // packThread has pushed a buffer and a nullptr to writeThread, remove
        // it
        while (fullList.front() != nullptr)
        {
            emptyList.push(fullList.front());
            fullList.pop();
        }
        fullList.pop();  // Remove nullptr
        fclose(file);
        throw runtime_error("Error creating write thread");
    }
    /*statsT =
        Thread::create(statsThreadLauncher, skywardStack(16 * 1024), 1, this,
    Thread::JOINABLE); if (!statsT)
    {
        fullQueue.put(nullptr);  // Signal packThread to stop
        packT->join();
        writeT->join();
        fclose(file);
        throw runtime_error("Error creating stats thread");
    }*/
    started     = true;
    s.opened    = true;
    s.logNumber = fileNumber;
    return fileNumber;
}

void Logger::stop()
{
    if (started == false)
        return;
    logStats();
    started = false;
    fullQueue.put(nullptr);  // Signal packThread to stop
    packT->join();
    writeT->join();
    // statsT->join();
    fclose(file);

    s.opened = false;
}

Logger::Logger()
{
    // Allocate buffers and put them in the empty list
    for (unsigned int i = 0; i < numBuffers; i++)
        emptyList.push(new Buffer);
    for (unsigned int i = 0; i < numRecords; i++)
        emptyQueue.put(new Record);
}

void Logger::packThreadLauncher(void* argv)
{
    reinterpret_cast<Logger*>(argv)->packThread();
}

void Logger::writeThreadLauncher(void* argv)
{
    reinterpret_cast<Logger*>(argv)->writeThread();
}

void Logger::statsThreadLauncher(void* argv)
{
    reinterpret_cast<Logger*>(argv)->statsThread();
}

LogResult Logger::logImpl(const char* name, const void* data, unsigned int size)
{
    if (started == false)
    {
        LOG_ERR(logger, "Logger not started!");
        ++s.statDroppedSamples;

        // Signal that we are trying to write to a closed log
        s.statWriteError = -1;

        return LogResult::Ignored;
    }

    Record* record = nullptr;
    {
        FastInterruptDisableLock dLock;
        // We disable interrupts because IRQget() is nonblocking, unlike get()
        if (emptyQueue.IRQget(record) == false)
        {
            s.statDroppedSamples++;
            return LogResult::Dropped;
        }
    }

    auto result = serializeImpl(record->data, maxRecordSize, name, data, size);
    if (result == BufferTooSmall)
    {
        emptyQueue.put(record);
        atomicAdd(&s.statTooLargeSamples, 1);
        return LogResult::TooLarge;
    }

    record->size = result;
    fullQueue.put(record);
    atomicAdd(&s.statQueuedSamples, 1);
    return LogResult::Queued;
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
            StackLogger::getInstance()->updateStack(THID_LOGGER_PACK);

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
                    s.statBufferFilled++;
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
                s.statBufferFilled++;
            }
        }
    }
    catch (exception& e)
    {
        printf("Error: packThread failed due to an exception: %s\n", e.what());
    }
}

void Logger::writeThread()
{
    try
    {
        for (;;)
        {
            StackLogger::getInstance()->updateStack(THID_LOGGER_WRITE);

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
            // green_led::high();

            size_t result = fwrite(buffer->data, 1, buffer->size, file);
            if (result != buffer->size)
            {
                // If this fails and your board uses SDRAM,
                // define and increase OVERRIDE_SD_CLOCK_DIVIDER_MAX
                // perror("fwrite");
                s.statWriteFailed++;
                s.statWriteError = ferror(file);
            }
            else
                s.statBufferWritten++;

            // green_led::low();
            timer.stop();
            s.statWriteTime    = timer.interval();
            s.statMaxWriteTime = max(s.statMaxWriteTime, s.statWriteTime);

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
        printf("Error: writeThread failed due to an exception: %s\n", e.what());
    }
}

/**
 * This thread prints stats
 */
void Logger::statsThread()
{
    try
    {
        for (;;)
        {
            Thread::sleep(1000);
            if (started == false)
                return;
            logStats();
            //             printf("ls:%d ds:%d qs:%d bf:%d bw:%d wf:%d wt:%d
            //             mwt:%d\n",
            //                    s.statTooLargeSamples, s.statDroppedSamples,
            //                    s.statQueuedSamples, s.statBufferFilled,
            //                    s.statBufferWritten,
            //                    s.statWriteFailed, s.statWriteTime,
            //                    s.statMaxWriteTime);
        }
    }
    catch (exception& e)
    {
        printf("Error: statsThread failed due to an exception: %s\n", e.what());
    }
}
