/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

namespace Boardcore
{

/**
 * @brief Wrapper around ActiveObject to help with lightweight tasks.
 */
class Tasklet : public ActiveObject
{
public:
    using StopFlag = std::atomic<bool>;
    using Function = std::function<void(StopFlag&)>;

    /**
     * @brief Create a tasklet.
     *
     * @param function Function to be called.
     * @param priority Priority of the tasklet.
     * @param stacksize Size of the stack of the tasklet.
     */
    explicit Tasklet(Function function,
                     miosix::Priority priority = miosix::MAIN_PRIORITY,
                     unsigned int stacksize = miosix::STACK_DEFAULT_FOR_PTHREAD)
        : ActiveObject(stacksize, priority), function(function)
    {
    }

private:
    void run() override;

    Function function;
};

/**
 * @brief Wrapper around ActiveObject to help with lightweight periodic tasks.
 */
class PeriodicTasklet : public ActiveObject
{
private:
    using Function = std::function<void()>;

    /**
     * @brief Task behavior policy.
     */
    enum class Policy
    {
        SKIP,    //< If the task misses it's deadline it's postponed at the next
                 // deadline.
        RECOVER  //< If the task misses it's deadline it's immediately
                 // rescheduled.
    };

    /**
     * @brief Create a preiodic tasklet.
     *
     * @param function Function to be called.
     * @param period Period of the tasklet.
     * @param policy Policy of the tasklet.
     * @param priority Priority of the tasklet.
     * @param stacksize Size of the stack of the tasklet.
     */
    explicit PeriodicTasklet(
        Function function, uint32_t period, Policy policy = Policy::RECOVER,
        miosix::Priority priority = miosix::MAIN_PRIORITY,
        unsigned int stacksize    = miosix::STACK_DEFAULT_FOR_PTHREAD)
        : ActiveObject(stacksize, priority), function(function), period(period),
          policy(policy)
    {
    }

private:
    void run() override;

    Function function;
    uint32_t period;
    Policy policy;
};

}  // namespace Boardcore