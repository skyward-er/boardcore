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

void EventScheduler::add(function_t func, uint32_t intervalMs) {
    task_t task = { func, intervalMs, false };
    addTask(task);
}

void EventScheduler::addOnce(function_t func, uint32_t delayMs) {
    task_t task = { func, delayMs, true };
    addTask(task);
}

void EventScheduler::run() {
    for(;;) {
        int64_t wakeupTime = -1;
        function_t func;
        
        {
            Lock<FastMutex> l(mutex);
            
            while(agenda.size() == 0) condvar.wait(mutex);
            
            int64_t now = getTick();
            if(agenda.top().nextTick <= now) {
                event_t e = agenda.top();
                agenda.pop();
                func = e.task->function;
                if(e.task->once==false) {
                    enqueue(e);
                    //updateStats(e,now);
                } else tasks.erase(e.task);
            } else {
                wakeupTime = agenda.top().nextTick;
            }
        }
        
        if(wakeupTime>0) Thread::sleepUntil(wakeupTime);
        if(func) func();
    }
}

void EventScheduler::addTask(const EventScheduler::task_t& task) {
    Lock<FastMutex> l(mutex);
    tasks.push_back(task);
    
    auto it = tasks.end();
    --it; //This makes it point to the last element of the list
    event_t event = { it, getTick() };
    enqueue(event);
}

void EventScheduler::enqueue(event_t& event) {
    event.nextTick += event.task->intervalMs * TICK_FREQ / 1000;
    agenda.push(event);
    condvar.broadcast();
}

EventScheduler::EventScheduler() : ActiveObject(1024) {}
