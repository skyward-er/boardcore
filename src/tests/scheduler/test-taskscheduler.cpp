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

using namespace Boardcore;
using namespace miosix;

GpioPin pin1 = GpioPin(GPIOB_BASE, 13);
GpioPin pin2 = GpioPin(GPIOB_BASE, 14);
GpioPin pin3 = GpioPin(GPIOB_BASE, 15);
GpioPin pin4 = GpioPin(GPIOD_BASE, 8);
GpioPin pin5 = GpioPin(GPIOD_BASE, 9);

void task2Hz()
{
    pin1.high();
    delayUs(1);
    pin1.low();

    printf("2 Hz tick\n");
}

void task5Hz()
{
    pin2.high();
    delayUs(1);
    pin2.low();
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

int main()
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

    TaskScheduler scheduler;

    TaskScheduler::function_t f2Hz{&task2Hz};
    TaskScheduler::function_t f5Hz{&task5Hz};
    TaskScheduler::function_t f500Hz{&task500Hz};
    TaskScheduler::function_t f1KHz{&task1KHz};
    TaskScheduler::function_t blockingF{&blockingTask};

    scheduler.addTask(f2Hz, 500, 1);
    scheduler.addTask(f5Hz, 200, 2);
    scheduler.addTask(f500Hz, 2, 3, TaskScheduler::Policy::RECOVER);
    scheduler.addTask(f1KHz, 1, 4);

    printf("4 tasks added (2Hz 5Hz 500Hz 1KHz)\n");
    printf("The scheduler will be started in 2 seconds\n");
    delayMs(2 * 1000);

    printf("Now starting the task scheduler\n");
    signalPin5();
    scheduler.start();

    Thread::sleep(4 * 1000);

    signalPin5();
    scheduler.removeTask(1);
    scheduler.removeTask(3);
    printf("Removed tasks 1 (2Hz) and 3 (500Hz)\n");

    Thread::sleep(4 * 1000);

    printf("Now readding task 1 and 3\n");
    signalPin5();
    scheduler.addTask(f2Hz, 1000 / 2, 1);
    scheduler.addTask(f500Hz, 1000 / 500, 3, TaskScheduler::Policy::RECOVER);

    Thread::sleep(4 * 1000);

    printf("Now adding a one shot task which will take 100ms to complete\n");
    signalPin5();
    scheduler.addTask(blockingF, 0, 5, TaskScheduler::Policy::ONE_SHOT);

    Thread::sleep(4 * 1000);

    signalPin5();
    scheduler.stop();
    printf("Task scheduler stopped\n");

    printf("Tasks stats:\n");
    for (auto stat : scheduler.getTaskStats())
    {
        printf("%d, %.2f, %.2f\n", stat.id, stat.periodStats.mean,
               stat.periodStats.stdev);
    }
}
