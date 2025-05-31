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
 * when the task should wake up and run. Use TimePoint::max() to indicate
 * that there is no deadline, and the task should not run until signaled.
 *
 * - task(): This function is invoked when the task wakes up after a deadline
 * expiration. Users should implement the logic that needs to be executed at
 * that time.
 */
class SignaledDeadlineTask : public ActiveObject
{
public:
    /**
     * The current version of the STL (GCC 9.2.0) has a bug where
     * condition_variable::wait_until is broken with clocks != system clock.
     *
     * When absolute times big enough are used, the resulting time point
     * will overflow, causing the condvar to return immediately.
     *
     * TODO: Use steady_clock once we update to a newer Miosix version using a
     * newer GCC version and therefore a newer STL.
     */
    using Clock     = std::chrono::system_clock;
    using TimePoint = std::chrono::time_point<Clock>;

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
     * @brief Signals the task to run and go to sleep until the next deadline.
     */
    void signalTask()
    {
        std::unique_lock<std::mutex> lock(mutex);
        signaled = true;
        condvar.notify_all();
    }

    /**
     * @brief Calculates the next deadline for the task to run.
     */
    virtual TimePoint nextTaskDeadline() = 0;

    /**
     * @brief The user-defined task to run.
     */
    virtual void task() = 0;

protected:
    void run() override
    {
        while (!shouldStop())
        {
            // Get the time when the task should run next
            auto deadline = nextTaskDeadline();

            // Lock the mutex in the smallest scope possible
            {
                std::unique_lock<std::mutex> lock(mutex);
                // Wait until the next deadline or until signaled
                condvar.wait_until(lock, deadline, [this] { return signaled; });
                // Reset the signaled flag
                signaled = false;
            }

            // Run the task
            task();
        }
    }

private:
    bool signaled = false;  ///< Avoid spurious wakeups
    std::mutex mutex;
    std::condition_variable condvar;
};
}  // namespace Boardcore
