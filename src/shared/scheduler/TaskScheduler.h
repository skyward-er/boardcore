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

#pragma once

#include <ActiveObject.h>
#include <Singleton.h>
#include <debug/debug.h>
#include <diagnostic/PrintLogger.h>
#include <units/Frequency.h>
#include <utils/KernelTime.h>
#include <utils/Stats/Stats.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <list>
#include <map>
#include <queue>

#include "TaskSchedulerData.h"

namespace Boardcore
{

/**
 * @brief The Task Scheduler allow to manage simple tasks with a single thread.
 * All the task added must not take more than 1ms to execute and should take
 * less time as possible to ensure other tasks are executed as they are supposed
 * to.
 *
 * HOW TO USE THE TASK SCHEDULER
 *
 * TaskScheduler.add(nonblocking_std::function_without_sleeps, millisec, id);
 * and.. it works like magic. :)
 *
 * Example:
 *    void magic_std::function() {
 *        // do something NONBLOCKING and WITHOUT SLEEPS
 *    }
 *    TaskScheduler.add(magic_std::function, 150);
 *
 */
class TaskScheduler : public ActiveObject
{
public:
    using function_t = std::function<void()>;

    /**
     * @brief The maximum number of tasks the scheduler can handle.
     */
    static constexpr size_t MAX_TASKS = 256;

    /**
     * @brief Task behavior policy.
     * Determines the behavior of the scheduler for a specific task.
     *
     * - ONE_SHOT: Runs the task once and subsequently removes it from the task
     * list. This is useful for one-off tasks.
     *
     * - SKIP: Skips missed executions. This is useful for tasks that need to
     * execute periodically but can skip some executions.
     * If the task misses one or more executions, the scheduler will skip the
     * missed executions, run the task once and re-schedule the task for
     * future execution. The scheduler will try to align the task execution time
     * with the original start time, but actual execution time is not guaranteed
     * to be aligned with the period.
     *
     * - RECOVER: Recovers missed executions. This is useful for
     * tasks that need to reach an overall number of iterations, but don't care
     * too much about timing.
     * Missed executions are recovered immediately, so this may cause one or
     * more tasks to clump at the beginning of the task queue until all missed
     * executions are recovered, causing the period to not be respected (see
     * issue #91).
     */
    enum class Policy : uint8_t
    {
        ONE_SHOT,  ///< Run the task one single timer.
        SKIP,      // Skips lost executions and stays aligned with the original
                   // start time.
        RECOVER    ///< Prioritize the number of executions over the period.
    };

    explicit TaskScheduler(miosix::Priority priority = miosix::PRIORITY_MAX -
                                                       1);

    /**
     * @brief Add a millisecond-period task function to the scheduler with an
     * auto generated ID.
     *
     * Note that each task has it's own unique ID, even one shot tasks!
     *
     * For one shot tasks, the period is used as a delay. If 0 the task will be
     * executed immediately, otherwise after the given period.
     *
     * @param function Function to be called periodically.
     * @param periodMs Inter call period [ms].
     * @param policy Task policy, default is RECOVER.
     * @param startTick Absolute system tick of the first activation, useful
     * for synchronizing tasks [ms]
     * @return The ID of the task if it was added successfully, 0 otherwise.
     */
    size_t addTask(function_t function, uint32_t periodMs,
                   Policy policy     = Policy::RECOVER,
                   int64_t startTick = Kernel::getOldTick())
    {
        auto period    = std::chrono::milliseconds{periodMs};
        auto startTime = std::chrono::time_point<std::chrono::steady_clock>{
            std::chrono::milliseconds{startTick}};

        return addTask(function, period, policy, startTime);
    }

    /**
     * @brief Add a task function with the given frequency to the scheduler with
     * an auto generated ID.
     *
     * Note that each task has it's own unique ID, even one shot tasks!
     *
     * For one shot tasks, the period is used as a delay. If 0 the task will be
     * executed immediately, otherwise after the given period.
     *
     * @param function Function to be called periodically.
     * @param frequency Task frequency [Hz].
     * @param policy Task policy, default is RECOVER.
     * @param startTime Absolute system time of the first activation, useful for
     * synchronizing tasks [ns]
     * @return The ID of the task if it was added successfully, 0 otherwise.
     */
    size_t addTask(function_t function, Units::Frequency::Hertz frequency,
                   Policy policy = Policy::RECOVER,
                   std::chrono::time_point<std::chrono::steady_clock>
                       startTime = std::chrono::steady_clock::now())
    {
        auto period = std::chrono::nanoseconds{
            static_cast<int64_t>(sToNs(1) / frequency.value())};

        return addTask(function, period, policy, startTime);
    }

    /**
     * @brief Add a task function with the given period to the scheduler with an
     * auto generated ID.
     *
     * Note that each task has it's own unique ID, even one shot tasks!
     *
     * For one shot tasks, the period is used as a delay. If 0 the task will be
     * executed immediately, otherwise after the given period.
     *
     * @param function Function to be called periodically.
     * @param period Inter call period [ns].
     * @param policy Task policy, default is RECOVER.
     * @param startTime Absolute system time of the first activation, useful for
     * synchronizing tasks [ns]
     * @return The ID of the task if it was added successfully, 0 otherwise.
     */
    size_t addTask(function_t function, std::chrono::nanoseconds period,
                   Policy policy = Policy::RECOVER,
                   std::chrono::time_point<std::chrono::steady_clock>
                       startTime = std::chrono::steady_clock::now());

    /**
     * @brief Enables the task with the given id.
     */
    void enableTask(size_t id);

    /**
     * @brief Disables the task with the given id, preventing it from executing.
     */
    void disableTask(size_t id);

    bool start() override;

    void stop() override;

    std::vector<TaskStatsResult> getTaskStats();

private:
    struct Task
    {
        function_t function;
        int64_t period;     ///< [ns]
        int64_t startTime;  ///< First activation time, useful for synchronizing
                            ///< tasks.
        bool enabled;       ///< Whether the task should be executed.
        Policy policy;
        int64_t lastCall;  ///< Last activation time for statistics computation.
        Stats activationStats;  ///< Stats about activation time error.
        Stats periodStats;      ///< Stats about period error.
        Stats workloadStats;    ///< Stats about time the task takes to compute.
        uint32_t missedEvents;  ///< Number of events that could not be run.
        uint32_t failedEvents;  ///< Number of events ended with exceptions.

        /**
         * @brief Default constructor that creates an empty, invalid task
         */
        Task();

        /**
         * @brief Creates a task with the given parameters
         *
         * @param function The std::function to be called
         * @param period The Period in [ns]
         * @param policy The task policy in case of a miss
         * @param startTime The first activation time
         */
        explicit Task(function_t function, int64_t period, Policy policy,
                      int64_t startTime);

        // Delete copy constructor and copy assignment operator to avoid copying
        // and force moving
        Task(const Task& other)            = delete;
        Task& operator=(const Task& other) = delete;

        // Define a move constructor and a move assignment operator to avoid
        // copying std::function
        Task(Task&& other)            = default;
        Task& operator=(Task&& other) = default;

        /**
         * @brief Checks if this task is empty.
         */
        bool empty() const { return !function; }
    };

    struct Event
    {
        size_t taskId;     ///< The task to execute.
        int64_t nextTime;  ///< Absolute time of next activation.

        Event(size_t taskId, int64_t nextTime)
            : taskId(taskId), nextTime(nextTime)
        {
        }

        /**
         * @brief Compare two events based on the next time.
         * @note This is used to have the event with the lowest time first in
         * the agenda. Newly pushed events are moved up in the queue (see
         * heap bubble-up) until the other time is lower.
         */
        bool operator>(const Event& other) const
        {
            return this->nextTime > other.nextTime;
        }
    };

    // Use `std::greater` as the comparator to have elements with the lowest
    // time first. Requires operator `>` to be defined for Event.
    using EventQueue =
        std::priority_queue<Event, std::vector<Event>, std::greater<Event>>;

    /**
     * @brief Instantiates a new EventQueue backed by a vector with a
     * capacity of `MAX_TASKS` to avoid reallocations when inserting new events.
     */
    static EventQueue makeAgenda();

    /**
     * @brief Populates the agenda prior to starting the scheduler. Checks the
     * start time of the tasks in the agenda and moves them in the future
     * respecting the period in respect to the original start time.
     * @note This function must be called before starting the scheduler or the
     * agenda will be empty.
     */
    void populateAgenda();

    void run() override;

    /**
     * @brief Update task statistics (Intended for when the task is executed).
     *
     * This function changes the task last call time to the startTime.
     *
     * \param event Current event.
     * \param startTime Start of execution time.
     * \param endTime End of execution time.
     */
    void updateStats(const Event& event, int64_t startTime, int64_t endTime);

    /**
     * @brief (Re)Enqueue an event into the agenda based on the scheduling
     * policy.
     *
     * Requires the mutex to be locked!
     *
     * \param event Event to be scheduled. Note: this parameter is modified, the
     * nextTime field is updated in order to respect the task interval.
     * \param startTime Activation time, needed to update the nextTime value of
     * the event.
     */
    void enqueue(Event event, int64_t startTime);

    static TaskStatsResult fromTaskIdPairToStatsResult(const Task& task,
                                                       size_t id)
    {
        return TaskStatsResult{id,
                               std::chrono::nanoseconds{task.period},
                               task.activationStats.getStats(),
                               task.periodStats.getStats(),
                               task.workloadStats.getStats(),
                               task.missedEvents,
                               task.failedEvents};
    }

    miosix::FastMutex mutex;  ///< Mutex to protect tasks and agenda.
    std::vector<Task> tasks;  ///< Holds all tasks to be scheduled, preallocated
                              ///< to MAX_TASKS avoid dynamic allocations.
    miosix::ConditionVariable condvar;  ///< Used when agenda is empty.
    EventQueue agenda;                  ///< Ordered list of functions.

    PrintLogger logger = Logging::getLogger("taskscheduler");
};

}  // namespace Boardcore
