/* Event scheduler
 *
 * Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Author: Alain Carlucci, Federico Terraneo, Matteo Michele Piazzolla
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Scheduler.h"

using namespace std;
using namespace miosix;

std::ostream& operator<<(std::ostream& os, const TaskStatResult& sr)
{
    os<<sr.name
      <<"\nactivation "<<sr.activationStats
      <<"\nperiod     "<<sr.periodStats
      <<"\nworkload   "<<sr.workloadStats<<'\n';
    return os;
}

//
// class EventScheduler
//

void EventScheduler::add(function_t func, uint32_t intervalMs, 
        const string& name, int64_t start) {
    task_t task = { func, intervalMs, name, false, -1 };
    addTask(task, start);
}

void EventScheduler::addOnce(function_t func, uint32_t delayMs, int64_t start) {
    task_t task = { func, delayMs, "", true, -1 };
    addTask(task, start);
}

vector<TaskStatResult> EventScheduler::getTaskStats()
{
    Lock<FastMutex> l(mutex);
    vector<TaskStatResult> result;
    result.reserve(permanentTasks);
    for(auto it : tasks)
    {
        if(it.once) continue;
        result.push_back(
            {
                it.name,
                it.activationStats.getStats(),
                it.periodStats.getStats(),
                it.workloadStats.getStats(),
            });
    }
    return result;
}

void EventScheduler::run() {
    Lock<FastMutex> l(mutex);
    for(;;) {
        while(agenda.size() == 0) condvar.wait(mutex);
        
        int64_t now = getTick();
        int64_t nextTick = agenda.top().nextTick;
        if(nextTick <= now) {
            event_t e = agenda.top();
            agenda.pop();
            
            {
                Unlock<FastMutex> u(l);
                #ifndef __NO_EXCEPTIONS
                try {
                #endif
                    e.task->function();
                #ifndef __NO_EXCEPTIONS
                } catch(...) {
                    //TODO: can't propagate exception or the event scheduler
                    //will stop working, but we may want to log it
                }
                #endif
            }
            
            if(e.task->once==false) {
                updateStats(e,now,getTick());
                //NOTE: enqueue writes in nextTick, so has to be called after
                enqueue(e);
            } else tasks.erase(e.task);
        } else {
            Unlock<FastMutex> u(l);
            Thread::sleepUntil(nextTick);
        }
    }
}

void EventScheduler::addTask(const EventScheduler::task_t& task, int64_t start) {
    Lock<FastMutex> l(mutex);
    tasks.push_back(task);
    if(task.once==false) permanentTasks++;
    
    auto it = tasks.end();
    --it; //This makes it point to the last element of the list
    event_t event = { it, start };
    enqueue(event);
}

void EventScheduler::enqueue(event_t& event) {
    event.nextTick += event.task->intervalMs * TICK_FREQ / 1000;
    agenda.push(event);
    condvar.broadcast();
}

void EventScheduler::updateStats(event_t& e, int64_t startTime, int64_t endTime)
{
    const float tickToMs = 1000.f / TICK_FREQ;
    
    //Activation stats
    float activationError = startTime - e.nextTick;
    e.task->activationStats.add(activationError * tickToMs);
    
    //Period stats
    int64_t last=e.task->lastcall;
    if(last>=0)
    {
        float period = startTime - last;
        e.task->periodStats.add(period * tickToMs);
    }
    e.task->lastcall = startTime;
    
    //Workload stats
    e.task->workloadStats.add(endTime - startTime);
}

EventScheduler::EventScheduler() : ActiveObject(1024,PRIORITY_MAX-1), permanentTasks(0) {}
