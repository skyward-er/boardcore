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
 
#ifndef EVENT_SCHEDULER_H
#define EVENT_SCHEDULER_H

#include <Common.h>
#include <Singleton.h>
#include <ActiveObject.h>
#include <queue>


/** 
 * HOW TO USE THE EVENT SCHEDULER
 * sEventScheduler->add(nonblocking_std::function_without_sleeps, millisec);
 * and.. it works like magic. :)
 *
 * Example: 
 *    void magic_std::function() { // do something NONBLOCKING and WITHOUT SLEEPS }
 *    sEventScheduler->add(magic_std::function, 150);
 */
class EventScheduler : Singleton<EventScheduler>, ActiveObject {
    friend class Singleton<EventScheduler>;
    typedef std::function<void()> function_t;

    /** 
     * std::function you want to call + timer
     */
    struct schedule_t {
        function_t function;  ///< Task function
        uint32_t intervalMs; ///< Task period
    };

    /**
     * A single event
     */
    struct event_t { 
        schedule_t schedule; ///< The task and period
        int64_t nextTick;   ///< Absolute time of next activation

        bool operator<(const event_t& e) const {
            return e.nextTick < nextTick;
        }
    };

public:
    /**
     * Add a task function to be called periodically by the scheduler
     * \param func function to be called
     * \param intervalMs inter call period
     */
    void add(function_t func, uint32_t intervalMs);

private:
    /**
     * Overrides ActiveObject::run()
     */
    void run();

    miosix::FastMutex mutex;             ///< Mutex to protect agenda
    miosix::ConditionVariable condvar;   ///< Used when agenda is empty
    std::priority_queue<event_t> agenda; ///< Ordered list of functions

    /**
     * (Re)Enqueue a schedule.
     * 
     * Requires the mutex to be locked.
     * \param event event to be scheduled. Note: this parameter is
     * modified, in detail the nextTick field is overvritten in
     * order to respect the task interval. This is done for
     * performance reason
     */
    void enqueue(event_t& event);
    
    /**
     * Constructor
     */
    EventScheduler();
};

#define sEventScheduler Singleton<EventScheduler>::getInstance()

#endif
