/* Event scheduler
 *
 * Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Author: Alain Carlucci
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

using miosix::Thread;
using miosix::TICK_FREQ;
using miosix::getTick;

using std::push_heap;
using std::pop_heap;

void __event_scheduler_thread(void *arg) {
    sEventScheduler->run();
}

EventScheduler::EventScheduler() {
    Thread::create(__event_scheduler_thread, 1024);
}

EventScheduler::~EventScheduler() {
    // This should never happen
}

void EventScheduler::run() {
    while(true) {
        mutex.lock();

        while(agenda.size() == 0)
            cond_var.wait(mutex);

        if(agenda[0].next_tick <= getTick()) {
            event_t e = agenda[0];
            pop_heap(agenda.begin(), agenda.end());
            agenda.pop_back();

            mutex.unlock();
            e.schedule.f();

            queue(e.schedule);
        } else {
            mutex.unlock();

            int64_t sleep_time = agenda[0].next_tick - getTick();

            if(sleep_time > 0)
                Thread::sleep(sleep_time);
        }
    }
}

void EventScheduler::queue(schedule_t schedule) {
    event_t event = { 
        schedule,
        getTick() + schedule.interval_ms * TICK_FREQ / 1000
    };

    mutex.lock();
    agenda.push_back(event);
    push_heap(agenda.begin(), agenda.end());
    mutex.unlock();
    cond_var.broadcast();
}

