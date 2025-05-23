/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Pietro Bortolus, Niccolò Betto
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

#include <chrono>
#include <condition_variable>
#include <mutex>

namespace Boardcore
{
/**
 * @brief A task that executes a user-defined function at specific time points,
 * or when signaled.
 *
 * This class is designed to be used in situations where a task needs to be
 * executed within specific deadlines, irregularly. The task will wake up at the
 * specified time points and execute the user-defined function. The task can
 * also be signaled to wake up and run immediately, regardless of the time
 * points.
 *
 * This class essentially emulates an EDF (Earliest Deadline First) scheduler on
 * a single thread. If a high-enough priority value is used, the task will be
 * scheduled as soon as the deadline is reached, this can be useful for hard
 * real-time tasks.
 *
 * Users must implement the following functions:
 *
 * - calculateNextDeadline(): This function should return the next time point
 * when the task should wake up and the update() function be called. It can
 * return a time point in the future or NoWakeup to indicate that the task does
 * not need to wake up anytime soon. After NoWakeup is returned, a call to
 * signal() is required to force the task to recompute the next wakeup time, as
 * the task will be put to sleep indefinitely.
 *
 * - update(): This function is invoked when the task wakes up after a
 * deadline expiration. Users should implement the logic that needs to be
 * executed at that time.
 */
class SignaledDeadlineTask : public ActiveObject
{
    using Clock                         = std::chrono::steady_clock;
    using TimePoint                     = std::chrono::time_point<Clock>;
    static constexpr TimePoint NoWakeup = TimePoint{};

public:
    /**
     * @brief Constructor.
     *
     * @param stackSize The size of the stack for the thread.
     * @param priority The priority of the thread.
     */
    SignaledDeadlineTask(
        unsigned int stackSize    = miosix::STACK_DEFAULT_FOR_PTHREAD,
        miosix::Priority priority = miosix::MAIN_PRIORITY)
        : ActiveObject(stackSize, priority)
    {
    }

    /**
     * @brief Signals the task to run and go to sleep until the next wakeup
     * time point.
     */
    void signal() { condvar.notify_all(); }

    /**
     * @brief Calculates the next wakeup time point when to execute the update
     * function.
     */
    virtual TimePoint calculateNextDeadline() = 0;

    /**
     * @brief The user-defined function to run.
     */
    virtual void update() = 0;

protected:
    void run() override
    {
        while (!shouldStop())
        {
            // Get the time when the update function should be called next
            auto wakeup = calculateNextDeadline();

            std::cv_status waitResult = std::cv_status::no_timeout;
            // Lock the mutex in the smallest scope possible
            {
                std::unique_lock<std::mutex> lock(mutex);
                if (wakeup == NoWakeup)
                    // Wait until the task is manually signaled
                    condvar.wait(lock);
                else
                    // Wait until the next wakeup time point or signaled
                    waitResult = condvar.wait_until(lock, wakeup);
            }

            // Call the user function
            update();
        }
    }

private:
    std::mutex mutex;
    std::condition_variable condvar;
};
}  // namespace Boardcore
