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
 
#ifndef EVENT_SCHEDULER_H
#define EVENT_SCHEDULER_H

#include <Common.h>
#include <Singleton.h>

using miosix::FastMutex;
using miosix::ConditionVariable;
using std::vector;
using std::function;

/** HOW TO USE THE EVENT SCHEDULER
 *  sEventScheduler->add(nonblocking_function_without_sleeps, millisec);
 *  and.. it works like magic. :)
 *
 *  Example: 
 *     void magic_function() { // do something NONBLOCKING and WITHOUT SLEEPS }
 *     sEventScheduler->add(magic_function, 150);
 */
class EventScheduler : Singleton<EventScheduler> {
    friend class Singleton<EventScheduler>;
    typedef function<void()> function_t;

    /** Function you want to call + timer */
    typedef struct {
        function_t f;
        uint32_t interval_ms;
    } schedule_t;

    /** A single event */
    typedef struct event_t { 
        schedule_t schedule;
        int64_t next_tick;

        bool operator<(const event_t& e) const {
            return e.next_tick < next_tick; // Reverse sort? Maybe :P
        }
    } event_t;
public:
    void add(function_t func, uint32_t interval_ms) {
        schedule_t s = { func, interval_ms };
        queue(s);
    }

    // Do not call this directly.
    void run();
private:
    /** This vector is a HEAP! (std::push_heap, std::pop_heap, ...) */
    vector<event_t> agenda;
    FastMutex mutex;
    ConditionVariable cond_var;

    // (Re)Enqueue a schedule
    void queue(schedule_t schedule);
    
    EventScheduler();
    ~EventScheduler();
};

#define sEventScheduler Singleton<EventScheduler>::getInstance()

#endif
