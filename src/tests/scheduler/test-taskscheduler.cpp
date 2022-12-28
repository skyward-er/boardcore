/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <scheduler/TaskScheduler.h>

namespace
{
using namespace Boardcore;
using namespace miosix;

GpioPin pin1 = GpioPin(GPIOB_BASE, 13);
GpioPin pin2 = GpioPin(GPIOB_BASE, 14);
GpioPin pin3 = GpioPin(GPIOB_BASE, 15);
GpioPin pin4 = GpioPin(GPIOD_BASE, 8);
GpioPin pin5 = GpioPin(GPIOD_BASE, 9);

bool taskLogEnabled;  ///< A flag to enable/disable task logging

void task2Hz()
{
    pin1.high();
    delayUs(1);
    pin1.low();

    if (taskLogEnabled)
    {
        printf("2 Hz tick\n");
    }
}

void task5Hz()
{
    pin2.high();
    delayUs(1);
    pin2.low();

    if (taskLogEnabled)
    {
        printf("5 Hz tick\n");
    }
}

void task500Hz()
{
    pin3.high();
    delayUs(1);
    pin3.low();
}

void task1KHz()
{
    pin4.high();
    delayUs(1);
    pin4.low();
}

void signalPin5()
{
    pin5.high();
    delayUs(1);
    pin5.low();
}

void blockingTask() { delayMs(100); }

// Shared task functions between tests
TaskScheduler::function_t f2Hz{&task2Hz};            ///< A 2 Hz task
TaskScheduler::function_t f5Hz{&task5Hz};            ///< A 5 Hz task
TaskScheduler::function_t f500Hz{&task500Hz};        ///< A 500 Hz task
TaskScheduler::function_t f1KHz{&task1KHz};          ///< A 1 KHz task
TaskScheduler::function_t blockingF{&blockingTask};  ///< A blocking task

/**
 * @brief Sets up the output pins
 */
void setup()
{
    pin1.mode(Mode::OUTPUT);
    pin1.low();
    pin2.mode(Mode::OUTPUT);
    pin2.low();
    pin3.mode(Mode::OUTPUT);
    pin3.low();
    pin4.mode(Mode::OUTPUT);
    pin4.low();
    pin5.mode(Mode::OUTPUT);
    pin5.low();

    taskLogEnabled = true;
}

void printTaskStats(TaskScheduler& scheduler)
{
    printf("Tasks stats:\n");
    for (auto stat : scheduler.getTaskStats())
    {
        printf("- %d:\n", stat.id);
        printf("\tActivation: %.2f, %.2f\n", stat.activationStats.mean,
               stat.activationStats.stdDev);
        printf("\tPeriod: %.2f, %.2f\n", stat.periodStats.mean,
               stat.periodStats.stdDev);
        printf("\tWorkload: %.2f, %.2f\n", stat.workloadStats.mean,
               stat.workloadStats.stdDev);
        printf("\tMissed events: %ld\n", stat.missedEvents);
        printf("\tFailed events: %ld\n", stat.failedEvents);
    }
}

/**
 * @brief Tests the most common usage patters of the scheduler
 */
void test_general_purpose()
{
    TaskScheduler scheduler{};

    int task1 = scheduler.addTask(f2Hz, 500);
    scheduler.addTask(f5Hz, 200);
    int task3 = scheduler.addTask(f500Hz, 2, TaskScheduler::Policy::RECOVER);
    scheduler.addTask(f1KHz, 1, TaskScheduler::Policy::RECOVER);
    scheduler.addTask(f1KHz, 1, TaskScheduler::Policy::RECOVER);

    printf("4 tasks added (2Hz 5Hz 500Hz 1KHz)\n");
    printf("The scheduler will be started in 2 seconds\n");
    delayMs(2 * 1000);

    printf("Now starting the task scheduler\n");
    signalPin5();
    if (!scheduler.start())
    {
        printf("Error starting the task scheduler\n");
        return;
    }

    Thread::sleep(4 * 1000);

    signalPin5();
    scheduler.disableTask(task1);
    scheduler.disableTask(task3);
    printf("Removed tasks 1 (2Hz) and 3 (500Hz)\n");

    Thread::sleep(4 * 1000);

    printf("Now re-adding task 1 and 3\n");
    signalPin5();
    scheduler.enableTask(task1);
    scheduler.enableTask(task3);

    Thread::sleep(4 * 1000);

    printf("Now adding a one shot task which will take 100ms to complete\n");
    signalPin5();
    scheduler.addTask(blockingF, 0, TaskScheduler::Policy::ONE_SHOT);

    Thread::sleep(4 * 1000);

    signalPin5();
    scheduler.stop();
    printf("Task scheduler stopped\n");

    printTaskStats(scheduler);
}

/**
 * @brief Tests scheduler with a full task list
 */
void test_fill_scheduler()
{
    TaskScheduler scheduler{};

    printf("Adding tasks until the scheduler is full\n");
    size_t taskCount = 0;
    // Fill up the scheduler with tasks
    do
    {
        size_t lastId =
            scheduler.addTask(f2Hz, 500, TaskScheduler::Policy::ONE_SHOT);
        if (!lastId)
        {
            break;
        }
        taskCount++;
    } while (true);

    // Subtract one because the 0-th task is reserved
    if (taskCount != TaskScheduler::MAX_TASKS - 1)
    {
        printf("Error: couldn't fill the scheduler: taskCount = %zu \n",
               taskCount);
        return;
    }

    printf("Done adding tasks: taskCount = %zu\n", taskCount);

    printf("Trying to add another task\n");
    // Try to add another task
    if (scheduler.addTask(f2Hz, 500, TaskScheduler::Policy::ONE_SHOT))
    {
        printf("Error: added a task when the scheduler was full\n");
        return;
    }

    printf("Added tasks successfully\n");

    printf("Starting the scheduler\n");
    scheduler.start();

    Thread::sleep(4 * 1000);

    printf("Stopping the scheduler\n");
    scheduler.stop();

    printTaskStats(scheduler);
}

/**
 * @brief Tests the addTask function while the scheduler is running
 */
void test_runtime_add_task()
{
    TaskScheduler scheduler{};

    scheduler.addTask(f2Hz, 500);
    scheduler.addTask(f500Hz, 2, TaskScheduler::Policy::RECOVER);

    printf("2 tasks added (2Hz 500Hz)\n");
    printf("Starting the scheduler\n");
    scheduler.start();

    Thread::sleep(4 * 1000);

    printf("Adding a new task (5Hz)\n");
    if (!scheduler.addTask(f5Hz, 200))
    {
        printf("Error adding task while the scheduler is running\n");
    }

    Thread::sleep(4 * 1000);

    printf("Stopping the scheduler\n");
    scheduler.stop();
}

/**
 * @brief Tests the enable/disable functions of the scheduler
 */
void test_enable_disable()
{
    TaskScheduler scheduler{};

    int task1 = scheduler.addTask(f2Hz, 500);
    scheduler.addTask(f5Hz, 200);
    int task3 = scheduler.addTask(f500Hz, 2, TaskScheduler::Policy::RECOVER);

    printf("3 tasks added (2Hz 5Hz 500Hz)\n");
    printf("Starting the scheduler\n");
    scheduler.start();

    Thread::sleep(4 * 1000);

    printf("Disabling task 1 (2Hz) and 3 (500Hz)\n");
    scheduler.disableTask(task1);
    scheduler.disableTask(task3);

    Thread::sleep(4 * 1000);

    printf("Re-enabling task 1 (2Hz) and 3 (500Hz)\n");
    scheduler.enableTask(task1);
    scheduler.enableTask(task3);

    printf("Disabling task 2 (5Hz)\n");

    Thread::sleep(2 * 1000);

    scheduler.stop();
    printf("Task scheduler stopped\n");

    printTaskStats(scheduler);
}

/**
 * @brief Tests various edge cases of the scheduler:
 * - Disabling out-of-range tasks
 * - Enabling out-of-range tasks
 * - Double start
 */
void test_edge_cases()
{
    TaskScheduler scheduler{};

    scheduler.addTask(f2Hz, 500);
    scheduler.addTask(f5Hz, 200);
    scheduler.addTask(f500Hz, 2, TaskScheduler::Policy::RECOVER);

    printf("3 tasks added (2Hz 5Hz 500Hz)\n");
    printf("Starting the scheduler\n");
    scheduler.start();

    printf("Starting the scheduler again");
    if (scheduler.start())
    {
        printf("Error: started the scheduler twice\n");
    }

    Thread::sleep(1000);

    printf("Disabling out-of-range tasks with IDs 0 and 256");
    scheduler.disableTask(0);
    scheduler.disableTask(256);

    Thread::sleep(1000);

    printf("Enabling out-of-range tasks with IDs 0 and 256");
    scheduler.enableTask(0);
    scheduler.enableTask(256);

    Thread::sleep(1000);

    printf("Stopping the scheduler\n");
    scheduler.stop();
}

/**
 * @brief Tests the scheduler with a general usage pattern for a prolonged
 * period of time
 */
void test_long_range()
{
    TaskScheduler scheduler{};

    scheduler.addTask(f2Hz, 500);
    scheduler.addTask(f5Hz, 200);
    scheduler.addTask(f500Hz, 2, TaskScheduler::Policy::RECOVER);

    printf("3 tasks added (2Hz 5Hz 500Hz)\n");
    printf("A message will be printed every minute\n");
    printf("Starting the scheduler\n");
    scheduler.start();

    for (int i = 0; i < 5; i++)
    {
        printf("Scheduler running for %d minute(s)\n", i);
        Thread::sleep(60 * 1000);
    }

    printf("Stopping the scheduler\n");
    scheduler.stop();
}

}  // namespace

int main()
{
    setup();

    // Avoid clutter from tasks since this test will add a lot of tasks
    taskLogEnabled = false;
    printf("=> Running the fill scheduler test\n");
    test_fill_scheduler();
    taskLogEnabled = true;

    printf("\n");

    printf("=> Running the runtime add task test\n");
    test_runtime_add_task();

    printf("\n");

    printf("=> Running the enable/disable test\n");
    test_enable_disable();

    printf("\n");

    printf("=> Running the edge cases tests\n");
    test_edge_cases();

    printf("\n");

    printf("=> Running the general purpose test\n");
    test_general_purpose();

    printf("\n");

    // Avoid clutter from tasks since this test will run for a while
    taskLogEnabled = false;
    printf("=> Running the long range test, this may take a while\n");
    test_long_range();
    taskLogEnabled = true;

    printf("\n");

    printf("\n=> Task Scheduler tests completed\n");
    return 0;
}
