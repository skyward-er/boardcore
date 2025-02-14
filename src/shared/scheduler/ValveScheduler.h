/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Pietro Bortolus
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

#include <condition_variable>

namespace Boardcore
{

/**
 * Utility class implementing the Active Object pattern
 * used to manage the scheduling of the valves.
 * This template class requires the implementation of the
 * updateNextActionTs() and updatePositions() methods.
 */
class valveScheduler : public ActiveObject
{
public:
    valveScheduler(unsigned int stacksize = miosix::STACK_DEFAULT_FOR_PTHREAD,
                   miosix::Priority priority = miosix::MAIN_PRIORITY)
        : ActiveObject(stacksize, priority)
    {
    }

protected:
    void run() override;

    /**
     * This function needs to be implemented by the derived class.
     * It should update the next action timestamp.
     */
    virtual void updateNextActionTs() = 0;

    /**
     * This function needs to be implemented by the derived class.
     * It should update the positions of the valves.
     */
    virtual void updatePositions() = 0;

private:
    // Mutex used to ensure the condition variable is handled correctly
    std::mutex conditionVariableMutex;

    std::condition_variable cv;

    // timestamp of next action that needs to be scheduled
    long long nextActionTs = 0;
};

}  // namespace Boardcore
