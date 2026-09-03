/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Skyward
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

/**
 * @file scheduler-show.cpp
 * @brief Showcase entrypoint: periodic tasks scheduled by the shared Boardcore
 * TaskScheduler, driving the compute unit LEDs and logging on the kernel
 * console through the shared PrintLogger.
 *
 * Target: stm32f767zi_skyward_compute_unit.
 *
 * Behavior:
 *  - a 4 Hz task advances a "chase" pattern across the four user LEDs
 *    (userLed1..userLed4);
 *  - a 1 Hz task logs a heartbeat with the current kernel tick.
 *
 * Build/flash/run:
 *  - ./sbs build scheduler-show
 *  - ./sbs flash -r scheduler-show
 *  - ./sbs run scheduler-show
 */

#include <cstdio>

#include "diagnostic/PrintLogger.h"
#include "miosix.h"
#include "scheduler/TaskScheduler.h"

#include <interfaces-impl/bsp_impl.h>

using namespace miosix;
using namespace Boardcore;

static PrintLogger logger = Logging::getLogger("scheduler-show");

// Current step of the LED chase pattern
static volatile unsigned int chaseStep = 0;

/**
 * Turns on a single LED selected by the chase step and all the others off.
 * The board bsp_impl.h exposes userLed4 but no led4On/led4Off helpers, so the
 * fourth LED is driven directly.
 */
__attribute__((noinline)) static void setChaseLed(unsigned int step)
{
    led1Off();
    led2Off();
    led3Off();
    userLed4::low();

    switch (step % 4)
    {
        case 0:
            led1On();
            break;
        case 1:
            led2On();
            break;
        case 2:
            led3On();
            break;
        default:
            userLed4::high();
            break;
    }
}

/**
 * Task run at 4 Hz: advance the chase pattern by one LED.
 */
__attribute__((noinline)) static void ledChaseTask()
{
    setChaseLed(++chaseStep);
}

/**
 * Task run at 1 Hz: heartbeat log with the kernel tick.
 */
__attribute__((noinline)) static void logHeartbeatTask()
{
    LOG_INFO(logger, "heartbeat: chase step {} at tick {}", chaseStep,
             miosix::getTime());
}

int main()
{
    // Logging is disabled by default in Release builds; enable it here so the
    // shared PrintLogger output is visible on the kernel console (USART1).
    Logging::getStdOutLogSink().enable();
    Logging::getStdOutLogSink().setLevel(LOGL_INFO);

    printf("Boardcore scheduler show: 4 Hz LED chase + 1 Hz heartbeat log\n");
    LOG_INFO(logger, "scheduler show started");

    TaskScheduler scheduler;
    scheduler.addTask(&ledChaseTask, 250, TaskScheduler::Policy::RECOVER);
    scheduler.addTask(&logHeartbeatTask, 1000, TaskScheduler::Policy::RECOVER);

    if (!scheduler.start())
    {
        LOG_ERR(logger, "failed to start the task scheduler");
        return 1;
    }

    LOG_INFO(logger, "task scheduler running");

    while (true)
        Thread::sleep(1000);
}
