/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Alain Carlucci, Federico Terraneo, Matteo Piazzolla
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

#include "TaskScheduler.h"

#include <diagnostic/SkywardStack.h>
#include <utils/TimeUtils.h>

#include <algorithm>
#include <mutex>

using namespace std;
using namespace std::chrono;
using namespace miosix;

namespace Boardcore
{

TaskScheduler::EventQueue TaskScheduler::makeAgenda()
{
    std::vector<Event> agendaStorage{};
    agendaStorage.reserve(MAX_TASKS);
    return EventQueue{std::greater<Event>{}, std::move(agendaStorage)};
}

TaskScheduler::TaskScheduler(miosix::Priority priority)
    : ActiveObject(STACK_MIN_FOR_SKYWARD, priority), tasks(),
      agenda(makeAgenda())
{
    // Preallocate the vector to avoid dynamic allocation later on
    tasks.reserve(MAX_TASKS);

    // Reserve the zeroth element so that the task ID is never zero, for
    // API compatibility: returned value from addTask() is zero on error.
    tasks.emplace_back();
}

size_t TaskScheduler::addTask(function_t function, nanoseconds period,
                              Policy policy, time_point<steady_clock> startTime)
{
    std::unique_lock<miosix::FastMutex> lock{mutex};

    if (tasks.size() >= MAX_TASKS)
    {
        // Unlock the mutex to release the scheduler resources before logging
        lock.unlock();
        LOG_ERR(logger, "Full task scheduler");
        return 0;
    }

    if (policy == Policy::ONE_SHOT)
    {
        startTime += period;
    }

    // Insert a new task with the given parameters
    tasks.emplace_back(function, period.count(), policy,
                       startTime.time_since_epoch().count());
    size_t id = tasks.size() - 1;

    // Only add the task to the agenda if the scheduler is running
    // Otherwise, the agenda will be populated when the scheduler is started
    if (isRunning())
    {
        agenda.emplace(id, startTime.time_since_epoch().count());
    }
    condvar.broadcast();  // Signals the run thread

    return id;
}

void TaskScheduler::enableTask(size_t id)
{
    std::unique_lock<miosix::FastMutex> lock{mutex};

    if (id > tasks.size() - 1)
    {
        lock.unlock();
        LOG_ERR(logger, "Tried to enable an out-of-range task, id = {}", id);
        return;
    }

    Task& task = tasks[id];

    // Check that the task function is not empty
    // Attempting to run an empty function will throw a bad_function_call
    // exception
    if (task.empty())
    {
        lock.unlock();
        LOG_WARN(logger, "Tried to enable an empty task, id = {}", id);
        return;
    }

    task.enabled = true;
    agenda.emplace(id, miosix::getTime() + task.period);
}

void TaskScheduler::disableTask(size_t id)
{
    std::unique_lock<miosix::FastMutex> lock{mutex};

    if (id > tasks.size() - 1)
    {
        lock.unlock();
        LOG_ERR(logger, "Tried to disable an out-of-range task, id = {}", id);
        return;
    }

    Task& task   = tasks[id];
    task.enabled = false;
    // Reset the last call time to avoid incorrect period statistics
    task.lastCall = -1;
}

bool TaskScheduler::start()
{
    Lock<FastMutex> lock(mutex);

    // This check is necessary to prevent task normalization if the scheduler is
    // already stopped
    if (running)
    {
        return false;
    }

    // Populate the agenda with the tasks we have so far
    populateAgenda();

    return ActiveObject::start();
}

void TaskScheduler::stop()
{
    stopFlag = true;      // Signal the run function to stop
    condvar.broadcast();  // Wake the run function even if there are no tasks

    ActiveObject::stop();
}

vector<TaskStatsResult> TaskScheduler::getTaskStats()
{
    Lock<FastMutex> lock(mutex);

    vector<TaskStatsResult> result;

    for (size_t id = 1; id < tasks.size(); id++)
    {
        const Task& task = tasks[id];
        if (task.enabled)
        {
            result.push_back(fromTaskIdPairToStatsResult(task, id));
        }
    }

    return result;
}

void TaskScheduler::populateAgenda()
{
    int64_t currentTime = miosix::getTime();

    for (size_t id = 1; id < tasks.size(); id++)
    {
        Task& task        = tasks[id];
        int64_t startTime = task.startTime;

        // Shift the task's start time if it precedes the current time
        // to avoid clumping all tasks at the beginning (see issue #91)
        if (startTime < currentTime)
        {
            int64_t timeSinceStart = currentTime - startTime;
            int64_t periodsMissed  = timeSinceStart / task.period;
            int64_t periodsToSkip  = periodsMissed + 1;
            startTime += periodsToSkip * task.period;
        }

        agenda.emplace(id, startTime);
    }
}

void TaskScheduler::run()
{
    Lock<FastMutex> lock(mutex);

    while (true)
    {
        while (agenda.empty() && !shouldStop())
        {
            condvar.wait(mutex);
        }

        // Exit if the ActiveObject has been stopped
        if (shouldStop())
        {
            return;
        }

        int64_t startTime = miosix::getTime();
        Event nextEvent   = agenda.top();

        if (nextEvent.nextTime <= startTime)
        {
            Task& nextTask = tasks[nextEvent.taskId];
            agenda.pop();

            // Execute the task function
            if (nextTask.enabled)
            {
                {
                    Unlock<FastMutex> unlock(lock);

                    try
                    {
                        nextTask.function();
                    }
                    catch (...)
                    {
                        // Update the failed statistic
                        nextTask.failedEvents++;
                    }
                }

                // Enqueue only on a valid task
                updateStats(nextEvent, startTime, miosix::getTime());
                enqueue(nextEvent, startTime);
            }
        }
        else
        {
            Unlock<FastMutex> unlock(lock);

            Thread::nanoSleepUntil(nextEvent.nextTime);
        }
    }
}

void TaskScheduler::updateStats(const Event& event, int64_t startTime,
                                int64_t endTime)
{
    Task& task = tasks[event.taskId];

    float activationTime = startTime - event.nextTime;
    task.activationStats.add(activationTime / Constants::NS_IN_MS);

    int64_t lastCall = task.lastCall;
    if (lastCall >= 0)
    {
        float periodTime = startTime - lastCall;
        task.periodStats.add(periodTime / Constants::NS_IN_MS);
    }
    // Update the last call time to the current start time for the next
    // iteration
    task.lastCall = startTime;

    float workloadTime = endTime - startTime;
    task.workloadStats.add(workloadTime / Constants::NS_IN_MS);
}

void TaskScheduler::enqueue(Event event, int64_t startTime)
{
    Task& task = tasks[event.taskId];
    switch (task.policy)
    {
        case Policy::ONE_SHOT:
            // If the task is one shot we won't push it to the agenda and we'll
            // remove it from the tasks map.
            task.enabled = false;
            return;
        case Policy::SKIP:
        {
            // Compute the number of missed periods since the last execution
            int64_t timeSinceLastExec = startTime - event.nextTime;
            int64_t periodsMissed     = timeSinceLastExec / task.period;

            // Schedule the task executon to the next aligned period, by
            // skipping over the missed ones
            // E.g. 3 periods have passed since last execution, the next viable
            // schedule time is after 4 periods
            int64_t periodsToSkip = periodsMissed + 1;
            // Update the task to run at the next viable timeslot, while still
            // being aligned to the original one
            event.nextTime += periodsToSkip * task.period;

            // Updated the missed events count
            task.missedEvents += static_cast<uint32_t>(periodsMissed);
            break;
        }
        case Policy::RECOVER:
            event.nextTime += task.period;
            break;
    }

    // Re-enqueue the event in the agenda and signals the run thread
    agenda.push(event);
    condvar.broadcast();
}

TaskScheduler::Task::Task()
    : function(nullptr), period(0), startTime(0), enabled(false),
      policy(Policy::SKIP), lastCall(-1), activationStats(), periodStats(),
      workloadStats(), missedEvents(0), failedEvents(0)
{
}

TaskScheduler::Task::Task(function_t function, int64_t period, Policy policy,
                          int64_t startTime)
    : function(function), period(period), startTime(startTime), enabled(true),
      policy(policy), lastCall(-1), activationStats(), periodStats(),
      workloadStats(), missedEvents(0), failedEvents(0)
{
}

}  // namespace Boardcore
