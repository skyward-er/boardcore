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

namespace Boardcore
{
namespace Constants
{
static constexpr unsigned int TICKS_PER_MS =
    miosix::TICK_FREQ / 1000;  // Number of ticks in a millisecond
static constexpr unsigned int MS_PER_TICK =
    1000 / miosix::TICK_FREQ;  // Number of milliseconds in a tick
}  // namespace Constants

TaskScheduler::TaskScheduler(miosix::Priority priority)
    : ActiveObject(STACK_MIN_FOR_SKYWARD, priority)
{
    // Create dynamically the tasks vector because of too much space
    tasks = new std::array<Task, TASKS_SIZE>();

    // Initialize the vector elements
    for (size_t i = 1; i < TASKS_SIZE; i++)
    {
        (*tasks)[i] = Task();
    }
}

TaskScheduler::~TaskScheduler() { delete tasks; }

size_t TaskScheduler::addTask(function_t function, uint32_t period,
                              Policy policy, int64_t startTick)
{
    Lock<FastMutex> lock(mutex);
    size_t id = 1;

    // Find a suitable id for the new task
    for (; id < TASKS_SIZE; id++)
    {
        if ((*tasks)[id].valid == false)
        {
            break;
        }
    }

    // Check if in the corresponding id there's already a task
    if ((*tasks)[id].valid)
    {
        // Unlock the mutex for expensive operation
        Unlock<FastMutex> unlock(mutex);

        LOG_ERR(logger, "Full task scheduler, id = {:zu}", id);
        return 0;
    }

    // Create a new task with the given parameters
    (*tasks)[id] = Task(function, period, policy);

    if (policy == Policy::ONE_SHOT)
    {
        startTick += period;
    }

    // Add the task first event in the agenda, performs in-place construction
    agenda.emplace(id, startTick);
    condvar.broadcast();  // Signals the run thread

    return id;
}

bool TaskScheduler::start()
{
    // This check is necessary to prevent task normalization if the scheduler is
    // already stopped
    if (running)
    {
        return false;
    }

    // Normalize the tasks start time if they precede the current tick
    normalizeTasks();

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

    for (size_t id = 1; id < TASKS_SIZE; id++)
    {
        const Task& task = (*tasks)[id];
        if (task.valid)
        {
            result.push_back(fromTaskIdPairToStatsResult(task, id));
        }
    }

    return result;
}

void TaskScheduler::normalizeTasks()
{
    int64_t currentTick = getTick();

    EventQueue newAgenda;
    while (!agenda.empty())
    {
        Event event = agenda.top();
        agenda.pop();

        if (event.nextTick < currentTick)
        {
            Task& task = (*tasks)[event.taskId];
            event.nextTick +=
                ((currentTick - event.nextTick) / task.period + 1) *
                task.period;
        }

        newAgenda.push(event);
    }
    agenda = std::move(newAgenda);
}

void TaskScheduler::run()
{
    Lock<FastMutex> lock(mutex);

    while (true)
    {
        while (agenda.empty() && !shouldStop())
            condvar.wait(mutex);

        // Exit if the ActiveObject has been stopped
        if (shouldStop())
        {
            return;
        }

        int64_t startTick = getTick();
        Event nextEvent   = agenda.top();
        Task& nextTask    = (*tasks)[nextEvent.taskId];

        // If the task has the SKIP policy and its execution was missed, we need
        // to move it forward to match the period
        if (nextEvent.nextTick < startTick && nextTask.policy == Policy::SKIP)
        {
            agenda.pop();
            enqueue(nextEvent, startTick);
        }
        else if (nextEvent.nextTick <= startTick)
        {
            agenda.pop();

            // Execute the task function
            if (nextTask.valid)
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
                updateStats(nextEvent, startTick, getTick());
                enqueue(nextEvent, startTick);
            }
        }
        else
        {
            Unlock<FastMutex> unlock(lock);

            Thread::sleepUntil(nextEvent.nextTick);
        }
    }
}

void TaskScheduler::updateStats(const Event& event, int64_t startTick,
                                int64_t endTick)
{
    Task& task = (*tasks)[event.taskId];

    // Activation stats
    float activationError = startTick - event.nextTick;
    task.activationStats.add(activationError * Constants::MS_PER_TICK);

    // Period stats
    int64_t lastCall = task.lastCall;
    if (lastCall >= 0)
        task.periodStats.add((startTick - lastCall) * Constants::MS_PER_TICK);

    // Update the last call tick to the current start tick for the next
    // iteration
    task.lastCall = startTick;

    // Workload stats
    task.workloadStats.add(endTick - startTick);
}

void TaskScheduler::enqueue(Event event, int64_t startTick)
{
    Task& task = (*tasks)[event.taskId];
    switch (task.policy)
    {
        case Policy::ONE_SHOT:
            // If the task is one shot we won't push it to the agenda and we'll
            // remove it from the tasks map.
            task.valid = false;
            return;
        case Policy::SKIP:
            // Updated the missed events count
            task.missedEvents += (startTick - event.nextTick) / task.period;

            // Compute the number of periods between the tick the event should
            // have been run and the tick it actually run. Than adds 1 and
            // multiply the period to get the next execution tick still aligned
            // to the original one.
            // E.g. If a task has to run once every 2 ticks and start at tick 0
            // but for whatever reason the first execution is at tick 3, then
            // the next execution will be at tick 4.
            event.nextTick +=
                ((startTick - event.nextTick) / task.period + 1) * task.period;
            break;
        case Policy::RECOVER:
            event.nextTick += task.period * Constants::TICKS_PER_MS;
            break;
    }

    // Re-enqueue the event in the agenda and signals the run thread
    agenda.push(event);
    condvar.broadcast();
}

TaskScheduler::Task::Task()
    : function(nullptr), period(0), valid(false), policy(Policy::SKIP),
      lastCall(-1), activationStats(), periodStats(), workloadStats(),
      missedEvents(0), failedEvents(0)
{
}

TaskScheduler::Task::Task(function_t function, uint32_t period, Policy policy)
    : function(function), period(period), valid(true), policy(policy),
      lastCall(-1), activationStats(), periodStats(), workloadStats(),
      missedEvents(0), failedEvents(0)
{
}

}  // namespace Boardcore
