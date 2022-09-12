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

TaskScheduler::TaskScheduler()
    : ActiveObject(STACK_MIN_FOR_SKYWARD, miosix::PRIORITY_MAX - 1)
{
}

uint8_t TaskScheduler::addTask(function_t function, uint32_t period, uint8_t id,
                               Policy policy, int64_t startTick)
{
    Lock<FastMutex> lock(mutex);

    // Register the task into the map
    Task task   = {function, period, id, policy, -1, {}, {}, {}, 0, 0};
    auto result = tasks.insert({id, task});

    if (policy == Policy::ONE_SHOT)
        startTick += period;

    if (result.second)
    {
        // Add the task first event in the agenda
        Event event = {&(result.first->second), startTick};
        agenda.push(event);
        condvar.broadcast();  // Signals the run thread
    }
    else
    {
        LOG_ERR(logger, "Trying to add a task which id is already present");
    }

    return result.second ? id : 0;
}

uint8_t TaskScheduler::addTask(function_t function, uint32_t period,
                               Policy policy, int64_t startTick)
{
    uint8_t id = 1;

    // Find a suitable id for the new task
    auto it = tasks.cbegin(), end = tasks.cend();
    for (; it != end && id == it->first; ++it, ++id)
        ;

    return addTask(function, period, id, policy, startTick) != 0 ? id : 0;
}

bool TaskScheduler::removeTask(uint8_t id)
{
    Lock<FastMutex> lock(mutex);

    if (tasks.find(id) == tasks.end())
    {
        LOG_ERR(logger, "Attempting to remove a task not registered");
        return false;
    }

    // Find the task in the agenda and remove it
    std::priority_queue<Event> newAgenda;
    while (agenda.size() > 0)
    {
        Event event = agenda.top();
        agenda.pop();

        if (event.task->id != id)
            newAgenda.push(event);
    }
    agenda = newAgenda;

    // Remove the task from the tasks map
    tasks.erase(id);

    return true;
}

bool TaskScheduler::start()
{
    // This check is necessary to prevent task normalization if the scheduler is
    // already stopped
    if (running)
        return false;

    // Normalize the tasks start time if they preceed the current tick
    normalizeTasks();

    return ActiveObject::start();
}

void TaskScheduler::stop()
{
    stopFlag = true;   // Signal the run function to stop
    condvar.signal();  // Wake the run function even if there are no tasks

    ActiveObject::stop();
}

vector<TaskStatsResult> TaskScheduler::getTaskStats()
{
    Lock<FastMutex> lock(mutex);

    vector<TaskStatsResult> result;

    std::transform(tasks.begin(), tasks.end(), std::back_inserter(result),
                   fromTaskIdPairToStatsResult);

    return result;
}

void TaskScheduler::normalizeTasks()
{
    int64_t currentTick = getTick();

    std::priority_queue<Event> newAgenda;
    while (agenda.size() > 0)
    {
        Event event = agenda.top();
        agenda.pop();

        if (event.nextTick < currentTick)
            event.nextTick +=
                ((currentTick - event.nextTick) / event.task->period + 1) *
                event.task->period;

        newAgenda.push(event);
    }
    agenda = newAgenda;
}

void TaskScheduler::run()
{
    Lock<FastMutex> lock(mutex);

    while (true)
    {
        while (agenda.size() == 0 && !shouldStop())
            condvar.wait(mutex);

        // Exit if the ActiveObject has been stopped
        if (shouldStop())
            return;

        int64_t startTick = getTick();
        Event nextEvent   = agenda.top();

        // If the task has the SKIP policy and its execution was missed, we need
        // to move it forward to match the period
        if (nextEvent.nextTick < startTick &&
            agenda.top().task->policy == Policy::SKIP)
        {
            agenda.pop();
            enqueue(nextEvent, startTick);
        }
        else if (nextEvent.nextTick <= startTick)
        {
            agenda.pop();

            // Execute the task function
            {
                Unlock<FastMutex> unlock(lock);

                try
                {
                    nextEvent.task->function();
                }
                catch (...)
                {
                    // Update the failed statistic
                    nextEvent.task->failedEvents++;
                }
            }

            updateStats(nextEvent, startTick, getTick());
            enqueue(nextEvent, startTick);
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
    const float tickToMs = 1000.f / TICK_FREQ;

    // Activation stats
    float activationError = startTick - event.nextTick;
    event.task->activationStats.add(activationError * tickToMs);

    // Period stats
    int64_t lastCall = event.task->lastCall;
    if (lastCall >= 0)
        event.task->periodStats.add((startTick - lastCall) * tickToMs);

    // Update the last call tick to the current start tick for the next
    // iteration
    event.task->lastCall = startTick;

    // Workload stats
    event.task->workloadStats.add(endTick - startTick);
}

void TaskScheduler::enqueue(Event& event, int64_t startTick)
{
    const float msToTick = TICK_FREQ / 1000.f;

    switch (event.task->policy)
    {
        case Policy::ONE_SHOT:
            // If the task is one shot we won't push it to the agenda and we'll
            // remove it from the tasks map.
            tasks.erase(event.task->id);
            return;
        case Policy::SKIP:
            // Updated the missed events count
            event.task->missedEvents +=
                (startTick - event.nextTick) / event.task->period;

            // Compute the number of periods between the tick the event should
            // have been run and the tick it actually run. Than adds 1 and
            // multiply the period to get the next execution tick still aligned
            // to the original one.
            // E.g. If a task has to run once every 2 ticks and start at tick 0
            // but for whatever reason the first execution is at tick 3, then
            // the next execution will be at tick 4.
            event.nextTick +=
                ((startTick - event.nextTick) / event.task->period + 1) *
                event.task->period;
            break;
        case Policy::RECOVER:
            event.nextTick += event.task->period * msToTick;
            break;
    }

    // Re-enqueue the event in the agenda and signals the run thread
    agenda.push(event);
    condvar.broadcast();
}

}  // namespace Boardcore
