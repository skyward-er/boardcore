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

#include <ActiveObject.h>
#include <Common.h>
#include <Singleton.h>
#include <math/Stats.h>
#include <list>
#include <queue>

/**
 * Statistics for a task
 */
struct TaskStatResult
{
    std::string name;             ///< Task name
    StatsResult activationStats;  ///< Task activation stats
    StatsResult periodStats;      ///< Task period stats
    StatsResult workloadStats;    ///< Task workload stats
};

/**
 * Allows printing TaskStatResult to an ostream
 */
std::ostream& operator<<(std::ostream& os, const TaskStatResult& sr);

/**
 * HOW TO USE THE EVENT SCHEDULER
 * sEventScheduler->add(nonblocking_std::function_without_sleeps, millisec);
 * and.. it works like magic. :)
 *
 * Example:
 *    void magic_std::function() {
 *        // do something NONBLOCKING and WITHOUT SLEEPS
 *    }
 *    sEventScheduler->add(magic_std::function, 150);
 *
 * **IMPORTANT REMINDER**
 * Tasks in the event scheduler are meant to be added at inizialization.
 * DO NOT add task with long intervals/delays BEFORE adding the ones with
 * shorter intervals/delays: when adding first a task with a long delay, other
 * tasks are not executed until the long interval has expired once, producing
 * unwanted delays.
 */
class EventScheduler : public Singleton<EventScheduler>, ActiveObject
{
    friend class Singleton<EventScheduler>;

public:
    typedef std::function<void()> function_t;

    /**
     * Add a task function to be called periodically by the scheduler
     * \param func function to be called
     * \param intervalMs inter call period
     * \param start the first activation will be at time start+intervalMs,
     * useful for synchronizing tasks
     */
    void add(function_t func, uint32_t intervalMs, const std::string& name,
             int64_t start = miosix::getTick());

    /**
     * Add a single shot task function to be called only once, after the
     * given delay
     * \param func function to be called
     * \param delayMs delay before the call
     * \param start the first activation will be at time start+intervalMs,
     * useful for synchronizing tasks
     */
    void addOnce(function_t func, uint32_t delayMs,
                 int64_t start = miosix::getTick());

    /**
     * \return statistics for all tasks
     */
    std::vector<TaskStatResult> getTaskStats();

private:
    /**
     * std::function you want to call + timer
     */
    struct task_t
    {
        function_t function;    ///< Task function
        uint32_t intervalMs;    ///< Task period
        std::string name;       ///< Task name
        bool once;              ///< true if the task is not periodic
        int64_t lastcall;       ///< Last activation for period computaton
        Stats activationStats;  ///< Stats about activation error
        Stats periodStats;      ///< Stats about period error
        Stats workloadStats;    ///< Stats about time the task takes to compute
    };

    /**
     * A single event
     */
    struct event_t
    {
        std::list<task_t>::iterator task;  ///< The task and period
        int64_t nextTick;                  ///< Absolute time of next activation

        bool operator<(const event_t& e) const
        {
            // Note: operator < is reversed, so that the priority_queue
            // will return the lowest element first
            return this->nextTick > e.nextTick;
        }
    };

    /**
     * Overrides ActiveObject::run()
     */
    void run();

    /**
     * Add a task to be executed, both periodic and single shot.
     * In addition, also takes care of genrating the (first) event for the task
     * \param pask the task to add
     */
    void addTask(const task_t& task, int64_t start);

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
     * Update task stats
     * \param e current event
     * \param startTime start of execution time
     * \param endTime end of execution time
     */
    void updateStats(event_t& e, int64_t startTime, int64_t endTime);

    /**
     * Constructor
     */
    EventScheduler();

    miosix::FastMutex mutex;              ///< Mutex to protect agenda
    miosix::ConditionVariable condvar;    ///< Used when agenda is empty
    std::list<task_t> tasks;              ///< Holds all tasks to be scheduled
    std::priority_queue<event_t> agenda;  ///< Ordered list of functions
    uint32_t permanentTasks;              ///< Number of non-oneshot tasks
};

#define sEventScheduler EventScheduler::getInstance()

#endif
