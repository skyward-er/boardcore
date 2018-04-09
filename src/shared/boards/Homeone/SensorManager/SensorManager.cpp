/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
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
#include "events/Scheduler.h"

namespace HomeoneBoard
{
namespace Sensors
{

SensorManager::SensorManager() : FSM(&SensorManager::handleEvent)
{
    sEventBroker->subscribe(this, TOPIC_SENSORS_SM);

    initSensors();
    initSamplers();
}

void SensorManager::initSensors() { test_sensor = new TestSensor(); }

void SensorManager::initSamplers() { m10HzSimple.AddSensor(test_sensor); }

void SensorManager::startSampling()
{
    // Simple 10 Hz Sampler callback and scheduler function
    std::function<void()> simple10HzCallback =
        std::bind(&SensorManager::onSimple10HZCallback, this);

    std::function<void()> simple10HzSampler = std::bind(
        &SimpleSensorSampler::Update, &m10HzSimple, simple10HzCallback);

    sEventScheduler->add(simple10HzSampler, 100, "simple_10hz");
}

void SensorManager::handleEvent(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_SM_START_SAMPLING:
            startSampling();
            break;
        default:
            printf("Unrecognized event\n");
    }
}

void SensorManager::onSimple10HZCallback()
{
    // This is just a test
    float value = test_sensor->getLastSample();
    printf("%f\n", value);

    // TODO: Send samples to logger
    // TODO: Send pressure samples to FMM
}
}
}
