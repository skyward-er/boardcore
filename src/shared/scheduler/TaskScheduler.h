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
#include <utils/Stats/Stats.h>

#include <cstdint>
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
    typedef std::function<void()> function_t;

    /**
     * @brief Task behavior policy.
     *
     * This policies allows to change the behavior of the scheduler for the
     * specific task:
     * - ONE_SHOT: Allows to run the task once. When it is executed, it is
     * removed from the tasks list.
     * - SKIP: If for whatever reason a task can't be executed when
     * it is supposed to (e.g. another thread occupies the CPU), the scheduler
     * doesn't recover the missed executions but instead skips those and
     * continues normally. This ensures that all the events are aligned with
     * the original start tick. In other words, the period and the start tick of
     * a task specifies the time slots the task has to be executed. If one of
     * this time slots can't be used, that specific execution won't be
     * recovered.
     * - RECOVER: On the other hand, the RECOVER policy ensures that the missed
     * executions are run. However, this will cause the period to not be
     * respected and the task will run consecutively for some time (See issue
     * #91).
     */
    enum class Policy
    {
        ONE_SHOT,  ///< Run the task one single timer.
        SKIP,      // Skips lost executions and stays aligned with the original
                   // start tick.
        RECOVER    ///< Prioritize the number of executions over the period.
    };

    TaskScheduler();

    /**
     * @brief Add a task function to the scheduler with an auto generated id.
     *
     * Note that each task has it's own unique ID, even one shot tasks!
     * Therefore, if a task already exists with the same id, the function will
     * fail and return false.
     *
     * For one shot tasks, the period is used as a delay. If 0 the task will be
     * executed immediately, otherwise after the given period.
     *
     * @param function Function to be called periodically.
     * @param period Inter call period.
     * @param policy Task policy, default is SKIP.
     * @param startTick First activation time, useful for synchronizing tasks.
     * @return true if the task was added successfully.
     */
    uint8_t addTask(function_t function, uint32_t period,
                    Policy policy     = Policy::SKIP,
                    int64_t startTick = miosix::getTick());

    /**
     * @brief Removes the task identified by the given id if it exists.
     *
     * @param id Id of the task to remove.
     * @return true if the task was removed.
     */
    bool removeTask(uint8_t id);

    bool start() override;

    void stop() override;

    std::vector<TaskStatsResult> getTaskStats();

private:
    /**
     * @brief Check the start time of the tasks in the agenda and moves them in
     * the future respecting the period in respect to the original start time.
     */
    void normalizeTasks();

    struct Task
    {
        function_t function;
        uint32_t period;
        uint8_t id;
        bool valid;
        Policy policy;
        int64_t lastCall;  ///< Last activation tick for statistics computation.
        Stats activationStats;  ///< Stats about activation tick error.
        Stats periodStats;      ///< Stats about period error.
        Stats workloadStats;    ///< Stats about time the task takes to compute.
        uint32_t missedEvents;  ///< Number of events that could not be run.
        uint32_t failedEvents;  ///< Number of events ended with exceptions.
    };

    struct Event
    {
        Task* task;        ///< The task to execute.
        int64_t nextTick;  ///< Tick of next activation.

        bool operator<(const Event& e) const
        {
            // Note: operator < is reversed, so that the priority_queue will
            // return the lowest element first
            return this->nextTick > e.nextTick;
        }
    };

    void run() override;

    /**
     * @brief Add a task function to the scheduler.
     *
     * Note that each task has it's own unique ID, even one shot tasks!
     * Therefore, if a task already exists with the same id, the function will
     * fail and return false.
     *
     * For one shot tasks, the period is used as a delay. If 0 the task will be
     * executed immediately, otherwise after the given period.
     *
     * @param function Function to be called periodically.
     * @param period Inter call period.
     * @param id Task identification number.
     * @param policy Task policy, default is SKIP.
     * @param startTick First activation time, useful for synchronizing tasks.
     * @return true if the task was added successfully.
     */
    uint8_t addTask(function_t function, uint32_t period, uint8_t id,
                    Policy policy     = Policy::SKIP,
                    int64_t startTick = miosix::getTick());

    /**
     * @brief Update task statistics (Intended for when the task is executed).
     *
     * This function changes the task last call tick to the startTick.
     *
     * \param event Current event.
     * \param startTick Start of execution tick.
     * \param endTick End of execution tick.
     */
    void updateStats(const Event& event, int64_t startTick, int64_t endTick);

    /**
     * @brief (Re)Enqueue an event into the agenda based on the scheduling
     * policy.
     *
     * Requires the mutex to be locked!
     *
     * \param event Event to be scheduled. Note: this parameter is modified, the
     * nextTick field is updated in order to respect the task interval.
     * \param startTick Activation tick, needed to update the nextTick value of
     * the event.
     */
    void enqueue(Event& event, int64_t startTick);

    static TaskStatsResult fromTaskIdPairToStatsResult(Task* task)
    {

        return TaskStatsResult{task->id,
                               task->period,
                               task->activationStats.getStats(),
                               task->periodStats.getStats(),
                               task->workloadStats.getStats(),
                               task->missedEvents,
                               task->failedEvents};
    }

    miosix::FastMutex mutex;            ///< Mutex to protect tasks and agenda.
    std::array<Task*, 256> tasks{};     ///< Holds all tasks to be scheduled.
    miosix::ConditionVariable condvar;  ///< Used when agenda is empty.
    std::priority_queue<Event> agenda;  ///< Ordered list of functions.

    PrintLogger logger = Logging::getLogger("taskscheduler");
};

}  // namespace Boardcore
