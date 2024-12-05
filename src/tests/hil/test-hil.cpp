/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <algorithms/AirBrakes/TrajectoryPoint.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <diagnostic/StackLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <events/utils/EventSniffer.h>
#include <hil/HIL.h>
#include <scheduler/TaskScheduler.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "Buses.h"
#include "HILSimulationConfig.h"
#include "Sensors/HILSensors.h"
#include "Sensors/Sensors.h"
#include "Sensors/SensorsConfig.h"

using namespace Boardcore;
using namespace HILConfig;
using namespace HILTest;

static const bool HIL_TEST = true;

int main()
{
    // Overall status, if at some point it becomes false, there is a problem
    // somewhere
    bool initResult    = true;
    PrintLogger logger = Logging::getLogger("main");
    Boardcore::TaskScheduler scheduler;

    // Create modules
    Buses* buses = new Buses();
    Sensors* sensors;

    if (HIL_TEST)
    {
        // Create hil modules
        auto* hilPhasesManager = new MainHILPhasesManager(
            []() { return Boardcore::TimedTrajectoryPoint(); });
        auto* hilTransceiver =
            new MainHILTransceiver(buses->usart2, hilPhasesManager);

        // Create HIL class where we specify how to use previous modules to
        // assemble ActuatorData
        MainHIL* hil = new MainHIL(
            hilTransceiver, hilPhasesManager,
            [&]()
            {
                auto actuatorData = ActuatorData();
                return actuatorData;
            },
            SIMULATION_PERIOD);

        if (HIL_TEST)
            initResult &= ModuleManager::getInstance().insert<MainHIL>(hil);
        sensors = new HILSensors(&scheduler, buses, hil, false);
    }
    else
    {
        sensors = new Sensors(&scheduler, buses);
    }

    // Insert modules
    initResult &= ModuleManager::getInstance().insert<Buses>(buses);
    initResult &= ModuleManager::getInstance().insert<Sensors>(sensors);

    // Start modules
    initResult &= EventBroker::getInstance().start();
    initResult &= scheduler.start();

    if (HIL_TEST)
    {
        initResult &= ModuleManager::getInstance().get<MainHIL>()->start();

        ModuleManager::getInstance().get<MainHIL>()->waitStartSimulation();
    }

    initResult &= ModuleManager::getInstance().get<Sensors>()->start();

    // Check the init result and launch an event
    printf(initResult ? "Init OK\n" : "Init ERROR\n");

    // Periodic statistics
    while (true)
    {
        Thread::sleep(1000);
        printf("max: %.3f, min: %.3f, free stack: %ld, free heap: %ld\n",
               CpuMeter::getCpuStats().maxValue,
               CpuMeter::getCpuStats().minValue,
               CpuMeter::getCpuStats().freeStack,
               CpuMeter::getCpuStats().freeHeap);
        CpuMeter::resetCpuStats();
        StackLogger::getInstance().log();
    }

    return 0;
}
