/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <diagnostic/StackLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <events/utils/EventSniffer.h>
#include <hil/Events.h>
#include <hil/HIL.h>
#include <hil/HILConfig.h>
#include <hil/HILFlightPhasesManager.h>
#include <scheduler/TaskScheduler.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "HILSimulationConfig.h"

using namespace Boardcore;
using namespace Common;
using namespace HILConfig;

int main()
{
    // Overall status, if at some point it becomes false, there is a problem
    // somewhere
    bool initResult    = true;
    PrintLogger logger = Logging::getLogger("main");

    // Create modules
    USART usart2(USART2, 115200);
    HIL* hil = new HIL(usart2, new HILFlightPhasesManager());
    Boardcore::TaskScheduler scheduler;

    // Insert modules
    if (!ModuleManager::getInstance().insert<HIL>(hil))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the HIL module");
    }

    // Start modules
    if (!EventBroker::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the EventBroker module");
    }

    if (!scheduler.start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the board scheduler module");
    }

    if (!ModuleManager::getInstance().get<HIL>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the HIL module");
    }

    scheduler.addTask(
        [&]()
        {
            HILConfig::SimulatorData* sensorData =
                ModuleManager::getInstance()
                    .get<HIL>()
                    ->simulator->getSensorData();
            HILConfig::ADAdataHIL adaDataHil{
                Boardcore::TimestampTimer::getTimestamp(),
                sensorData->gps.positionMeasures[0][2],
                sensorData->gps.velocityMeasures[0][2]};
            HILConfig::ActuatorData actuatorData{
                NASState{},
                adaDataHil,
                0,
                30,
                sensorData->flags.flag_ascent,
                sensorData->flags.flag_burning};

            // Sending to the simulator
            ModuleManager::getInstance().get<HIL>()->send(actuatorData);
        },
        HILConfig::SIMULATION_PERIOD);

    // scheduler.addTask(
    //     [&]() {
    //     ModuleManager::getInstance().get<HIL>()->simulator->getSensorData()->print();
    //     }, 1000);

    // Check the init result and launch an event
    if (initResult)
    {
        // Post OK
        LOG_INFO(logger, "Init OK");

        // Set the LED status
        miosix::ledOn();
    }
    else
    {
        LOG_ERR(logger, "Init ERROR");
    }

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