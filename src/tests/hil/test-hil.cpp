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

#include "HILSimulationConfig.h"

using namespace Boardcore;
using namespace HILConfig;

static const bool HIL_TEST = true;

int main()
{
    // Overall status, if at some point it becomes false, there is a problem
    // somewhere
    bool initResult    = true;
    PrintLogger logger = Logging::getLogger("main");
    Boardcore::TaskScheduler scheduler;

    // Create modules
    USART usart2(USART2, SIM_BAUDRATE);

    // Create hil modules
    auto* hilTransceiver   = new MainHILTransceiver(usart2);
    auto* hilPhasesManager = new MainHILPhasesManager(
        []() { return Boardcore::TimedTrajectoryPoint(); });

    auto* sensorData = hilTransceiver->getSensorData();

    auto* accelerometer = new MainHILAccelerometer(&sensorData->accelerometer);
    auto* gyroscope     = new MainHILGyroscope(&sensorData->gyro);
    auto* magnetometer  = new MainHILMagnetometer(&sensorData->magnetometer);
    auto* gps           = new MainHILGps(&sensorData->gps);
    auto* barometer1    = new MainHILBarometer(&sensorData->barometer1);
    auto* barometer2    = new MainHILBarometer(&sensorData->barometer2);
    auto* barometer3    = new MainHILBarometer(&sensorData->barometer3);
    auto* baroChamber   = new MainHILBarometer(&sensorData->pressureChamber);
    auto* pitot         = new MainHILPitot(&sensorData->pitot);
    auto* temperature   = new MainHILTemperature(&sensorData->temperature);

    // Create HIL class where we specify how to use previous modules to assemble
    // ActuatorData
    MainHIL* hil = new MainHIL(
        hilTransceiver, hilPhasesManager,
        [&]()
        {
            auto actuatorData = ActuatorData();

            // ada
            // const auto gpsSample                 = gps->getLastSample();
            // actuatorData.adaState.aglAltitude    = gpsSample.height;
            // actuatorData.adaState.mslAltitude    = gpsSample.height - 160;
            // actuatorData.adaState.verticalSpeed  = -gpsSample.velocityDown;
            // actuatorData.adaState.apogeeDetected = static_cast<float>(false);
            // actuatorData.adaState.updating       = static_cast<float>(true);

            actuatorData.flags = sensorData->flags;
            return actuatorData;
        },
        SIMULATION_PERIOD);

    // Insert modules
    if (!ModuleManager::getInstance().insert<MainHIL>(hil))
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

    if (HIL_TEST)
    {
        if (!ModuleManager::getInstance().get<MainHIL>()->start())
        {
            initResult = false;
            LOG_ERR(logger, "Error starting the HIL module");
        }

        ModuleManager::getInstance().get<MainHIL>()->waitStartSimulation();
    }

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