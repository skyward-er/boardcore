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

#include <algorithm>

using namespace std;
using namespace miosix;
using namespace Boardcore::Constants;

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

size_t TaskScheduler::addTask(function_t function, uint32_t period,
                              Policy policy, int64_t startTime)
{
    // In the case of early returns, using RAII mutex wrappers to unlock the
    // mutex would cause it to be locked and unlocked one more time before
    // returning, because of the destructor being called on the Unlock object
    // first and then on the Lock object. To avoid this, we don't use RAII
    // wrappers and manually lock and unlock the mutex instead.
    mutex.lock();

    if (tasks.size() >= MAX_TASKS)
    {
        // Unlock the mutex to release the scheduler resources before logging
        mutex.unlock();
        LOG_ERR(logger, "Full task scheduler");
        return 0;
    }

    auto periodNs = period * NS_IN_MS;

    if (policy == Policy::ONE_SHOT)
    {
        startTime += periodNs;
    }

    // Insert a new task with the given parameters
    tasks.emplace_back(function, periodNs, policy, startTime);
    size_t id = tasks.size() - 1;

    // Only add the task to the agenda if the scheduler is running
    // Otherwise, the agenda will be populated when the scheduler is started
    if (isRunning())
    {
        agenda.emplace(id, startTime);
    }
    condvar.broadcast();  // Signals the run thread

    mutex.unlock();
    return id;
}

void TaskScheduler::enableTask(size_t id)
{
    mutex.lock();

    if (id > tasks.size() - 1)
    {
        mutex.unlock();
        LOG_ERR(logger, "Tried to enable an out-of-range task, id = {}", id);
        return;
    }

    Task& task = tasks[id];

    // Check that the task function is not empty
    // Attempting to run an empty function will throw a bad_function_call
    // exception
    if (task.empty())
    {
        mutex.unlock();
        LOG_WARN(logger, "Tried to enable an empty task, id = {}", id);
        return;
    }

    task.enabled = true;
    agenda.emplace(id, getTime() + task.period);
    mutex.unlock();
}

void TaskScheduler::disableTask(size_t id)
{
    mutex.lock();

    if (id > tasks.size() - 1)
    {
        mutex.unlock();
        LOG_ERR(logger, "Tried to disable an out-of-range task, id = {}", id);
        return;
    }

    tasks[id].enabled = false;
    mutex.unlock();
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
    int64_t currentTime = getTime();

    for (size_t id = 1; id < tasks.size(); id++)
    {
        Task& task = tasks[id];

        int64_t nextTime = task.startTime;
        // Normalize the tasks start time if they precede the current time
        if (nextTime < currentTime)
        {
            int64_t timeSinceStart = currentTime - nextTime;
            int64_t periodsMissed  = timeSinceStart / task.period;
            int64_t periodsToSkip  = periodsMissed + 1;
            nextTime += periodsToSkip * task.period;
        }

        agenda.emplace(id, nextTime);
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

        int64_t startTime = getTime();
        Event nextEvent   = agenda.top();
        Task& nextTask    = tasks[nextEvent.taskId];

        // If the task has the SKIP policy and its execution was missed, we need
        // to move it forward to match the period
        if (nextEvent.nextTime < startTime && nextTask.policy == Policy::SKIP)
        {
            agenda.pop();
            enqueue(nextEvent, startTime);
        }
        else if (nextEvent.nextTime <= startTime)
        {
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
                updateStats(nextEvent, startTime, getTime());
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

    // Activation stats
    float activationError = startTime - event.nextTime;
    task.activationStats.add(activationError / NS_IN_MS);

    // Period stats
    int64_t lastCall = task.lastCall;
    if (lastCall >= 0)
        task.periodStats.add((startTime - lastCall) / NS_IN_MS);

    // Update the last call time to the current start time for the next
    // iteration
    task.lastCall = startTime;

    // Workload stats
    task.workloadStats.add((endTime - startTime) / NS_IN_MS);
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
