/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#include "Common.h"
#include "boards/Homeone/Events.h"
#include "events/EventBroker.h"
#include "boards/Homeone/SensorManager/SensorManager.h"
#include "events/Scheduler.h"

using namespace miosix;
using namespace HomeoneBoard;

int main()
{
    TRACE("DEBUG\n");
    // Start active objects
    sEventBroker->start();
    sEventScheduler->start();

    SensorManager& mgr = *SensorManager::getInstance();
    mgr.start();
    printf("Current State: %d\n", mgr.getStatus().state);
    printf("Problematic sensors: %d\n\n", mgr.getStatus().problematic_sensors);

    Thread::sleep(2000);
    //sEventBroker->post({EV_TC_START_SAMPLING}, TOPIC_CONFIGURATION);
    Thread::sleep(500);
    printf("Current State: %d\n\n", mgr.getStatus().state);

    Thread::sleep(3000);

    for (;;)
    {
        printf("**STATS**\n");
        std::vector<TaskStatResult> stats = mgr.getSchedulerStats();
        for (TaskStatResult stat : stats)
        {
            printf("%s\n", stat.name.c_str());
            printf("Activation:\tmax:%.2f,\tmin:%.2f,\tmean:%.2f\n",
                   stat.activationStats.maxValue, stat.activationStats.minValue,
                   stat.activationStats.mean);
            printf("Period:   \tmax:%.2f,\tmin:%.2f,\tmean:%.2f\n",
                   stat.periodStats.maxValue, stat.periodStats.minValue,
                   stat.periodStats.mean);
            printf("Workload:\tmax:%.2f,\tmin:%.2f,\tmean:%.2f\n\n",
                   stat.workloadStats.maxValue, stat.workloadStats.minValue,
                   stat.workloadStats.mean);
        }
        Thread::sleep(10000);
    }
}
