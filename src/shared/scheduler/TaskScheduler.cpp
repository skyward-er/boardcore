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

bool TaskScheduler::addTask(function_t function, uint32_t period, uint8_t id,
                            Policy policy, int64_t startTick)
{
    // Perion must be grather than zero!
    if (period <= 0)
        return false;

    Lock<FastMutex> lock(mutex);

    // Register the task into the map
    Task task   = {function, period, id, policy, -1, 0, {}, {}, {}};
    auto result = tasks.insert({id, task});

    if (result.second)
    {
        // Add the task first event in the agenda
        Event event = {&(result.first->second), startTick};
        agenda.push(event);
        condvar.broadcast();  // Signals the run thread
    }

    return result.second;
}

bool TaskScheduler::removeTask(uint8_t id)
{
    Lock<FastMutex> lock(mutex);

    auto result = tasks.find(id);

    if (result == tasks.end())
        return false;

    result->second.policy = Policy::ONE_SHOT;

    return true;
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

void TaskScheduler::run()
{
    Lock<FastMutex> lock(mutex);

    // First of all check if the tasks start tick precedes the current tick.
    // This is to prevent unwanted

    while (true)
    {
        while (agenda.size() == 0 && !shouldStop())
            condvar.wait(mutex);

        // Exit if the ActiveObject has been stopped.
        if (shouldStop())
            return;

        int64_t startTick = getTick();
        int64_t nextTick  = agenda.top().nextTick;

        if (nextTick <= startTick)
        {
            Event event = agenda.top();
            agenda.pop();

            // Execute the task function
            {
                Unlock<FastMutex> unlock(lock);

                try
                {
                    event.task->function();
                }
                catch (...)
                {
                    // Update the failed statistic
                    event.task->failedEvents++;
                }
            }

            // Update the task statistics.
            updateStats(event, startTick, getTick());

            // Re-enqueue the task in the agenda based on the scheduling policy
            enqueue(event, startTick);
        }
        else
        {
            Unlock<FastMutex> unlock(lock);
            Thread::sleepUntil(nextTick);
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
