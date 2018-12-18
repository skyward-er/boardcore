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

#include "SensorManager.h"

#include "TestSensor.h"
#include "boards/Homeone/Events.h"
#include "boards/Homeone/Topics.h"
#include "events/EventBroker.h"

#include "drivers/adc/AD7994.h"
#include "sensors/ADIS16405.h"
#include "sensors/MAX21105.h"

#include "sensors/MPU9250/MPU9250.h"
#include "sensors/MPU9250/MPU9250Data.h"

#include "Debug.h"

using miosix::FastMutex;
using miosix::Lock;

namespace HomeoneBoard
{

SensorManager::SensorManager()
    : FSM(&SensorManager::stateIdle), logger(*LoggerProxy::getInstance())
{
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TC);

    initSensors();
    initSamplers();
}

void SensorManager::initSensors()
{
    sensor_test = new TestSensor();  // TODO: Remove this

    adc_ad7994 = new AD7994Type(AD7994_I2C_ADDRESS);

    imu_max21105 =
        new MAX21105Type(0, 0);  // TODO: Update with correct parameters
    if (!imu_max21105->init())
    {
        status.problematic_sensors |= SENSOR_MAX21105;
    }

    imu_mpu9250 =
        new MPU9250Type(0, 0);  // TODO: Update with correct parameters
    if (!imu_mpu9250->init())
    {
        status.problematic_sensors |= SENSOR_MPU9255;
    }

    // imu_adis16405 = new ADIS16405Type();
    // imu_adis16405->init();
}

void SensorManager::initSamplers()
{
    sampler_20hz_simple.AddSensor(sensor_test);
    sampler_20hz_simple.AddSensor(adc_ad7994);

    /*sampler_500hz_dma.AddSensor(imu_max21105);
    sampler_500hz_dma.AddSensor(imu_mpu9250);
    sampler_500hz_dma.AddSensor(imu_adis16405);*/
}

void SensorManager::stateIdle(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            enable_sensor_logging = false;

            TRACE("SM: Entering stateIdle\n");
            status.state = SensorManagerState::IDLE;
            logger.log(status);
            break;
        case EV_EXIT:
            break;

        // Perform the transition in both cases
        case EV_TC_START_LOGGING:
        case EV_ARMED:
            transition(&SensorManager::stateLogging);
            break;

        default:
            break;
    }
}

void SensorManager::stateLogging(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            enable_sensor_logging = true;

            TRACE("SM: Entering stateLogging\n");
            status.state = SensorManagerState::LOGGING;
            logger.log(status);
            break;
        case EV_EXIT:
            break;

        // Go back to idle in both cases
        case EV_TC_STOP_LOGGING:
        case EV_LANDED:
            transition(&SensorManager::stateIdle);
            break;

        default:
            break;
    }
}

void SensorManager::startSampling()
{
    /*
     * std::bind syntax:
     * std::bind(&MyClass::someFunction, &myclass_instance, [someFunction args])
     */

    // Simple 20 Hz Sampler callback and scheduler function
    std::function<void()> simple_20hz_callback =
        std::bind(&SensorManager::onSimple20HZCallback, this);
    std::function<void()> simple_20hz_sampler =
        std::bind(&SimpleSensorSampler::UpdateAndCallback, &sampler_20hz_simple,
                  simple_20hz_callback);

    sEventScheduler->add(simple_20hz_sampler, 500,
                         "simple_20hz");  // TODO: back to 50

    // DMA 500 Hz Sampler callback and scheduler function
    std::function<void()> dma_500hz_callback =
        std::bind(&SensorManager::onDMA500HZCallback, this);
    std::function<void()> dma_500Hz_sampler =
        std::bind(&DMASensorSampler::UpdateAndCallback, &sampler_500hz_dma,
                  dma_500hz_callback);

    sEventScheduler->add(dma_500Hz_sampler, 1000,
                         "dma_500hz");  // TODO: Back to 4 ms

    // Lambda expression callback to log scheduler stats, at 5 Hz
    sEventScheduler->add(
        [&]() {
            scheduler_stats = sEventScheduler->getTaskStats();

            for (TaskStatResult stat : scheduler_stats)
                logger.log(stat);
        },
        200, "stats");

    TRACE("Scheduler initialization complete\n");
}

void SensorManager::onSimple20HZCallback()
{
    // This is just a test
    printf("SIMPLE: %f\n", *(sensor_test->testDataPtr()));

    // TODO: Send samples to logger
    // TODO: Send pressure samples to FMM
}

void SensorManager::onDMA500HZCallback()
{
    printf("DMA: %f\n", *(sensor_test->testDataPtr()));
    /*MAX21105Data max21105_data{*(imu_max21105->accelDataPtr()),
     *(imu_max21105->gyroDataPtr()),
     *(imu_max21105->tempDataPtr())};*/

    /* MPU9250Data mpu9255_data{
     *(imu_mpu9250->accelDataPtr()), *(imu_mpu9250->gyroDataPtr()),
     *(imu_mpu9250->compassDataPtr()), *(imu_mpu9250->tempDataPtr())};*/

    // logger.log(mpu9255_data);

    /*  log.log(*(imu_adis16405->gyroDataPtr()));
      log.log(*(imu_adis16405->accelDataPtr()));
      log.log(*(imu_adis16405->tempDataPtr()));*/
}

}  // namespace HomeoneBoard
