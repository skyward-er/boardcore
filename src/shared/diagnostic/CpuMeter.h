/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Author: Federico Terraneo
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

#include <utils/Stats/Stats.h>

namespace Boardcore
{

namespace CpuMeter
{

/*
 * This CPU meter works like this.
 * It creates a thread with the minimum priority that is (almost) always active
 * Since the Miosix priority scheduler always runs the thread with the highest
 * priority, every time other threads (or interrupts) have work to do, they
 * preempt this thread. By measuring the amount of actual time this thread
 * takes to perform a busy wait delay it is possible to compute an average CPU
 * utilization.
 *
 * The advantage of this technique is that it can account for the CPU time
 * of everything, including interrupts and task context switch overhead.
 * The disadvantage is that by being (almost) always running it prevents the
 * idle thread from running and thus it prevents the CPU from going into deep
 * sleep.
 *
 * NOTE: For this to work, no other thread with the lowest priority has to be
 * created, otherwise its time will not be accounted.
 */

/// If defined, the CPU meter is activated
#define ENABLE_CPU_METER

/**
 * \return the average CPU utilization
 */
StatsResult averageCpuUtilization();

}  // namespace CpuMeter

}  // namespace Boardcore
