/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <ActiveObject.h>
#include <Singleton.h>
#include <logger/Logger.h>

#include <map>

#include "StackData.h"

using miosix::FastMutex;
using miosix::Lock;

namespace Boardcore
{

/**
 * Log the absolute free stack of each thread 1 times per second
 * StackLogger::update(id) must be called periodically from each thread in order
 * for data to be updated
 */
class StackLogger : public Singleton<StackLogger>
{
    friend class Singleton<StackLogger>;

public:
    /**
     * Update the stored absolute free stack associated to the calling thread.
     * This function must be called by the thread to be logged.
     * @param threadId Unique id identifying the thread
     */
    void updateStack(uint8_t threadId)
    {
        StackData d;
        d.timestamp    = miosix::IRQgetTime() / 1e6;
        d.threadId     = threadId;
        d.minimumStack = miosix::MemoryProfiling::getAbsoluteFreeStack();

        {
            Lock<FastMutex> l(mutex);
            stacks[threadId] = d;
        }
    }

    /**
     * Logs the most up-to-date stack data
     * Call this function periodically
     */
    void log()
    {
        {
            Lock<FastMutex> l(mutex);
            for (auto it = stacks.begin(); it != stacks.end(); it++)
            {
                Logger::getInstance().log(it->second);
            }
        }
    }

private:
    StackLogger() {}
    ~StackLogger() {}

    FastMutex mutex;
    std::map<uint8_t, StackData> stacks;
};

}  // namespace Boardcore
