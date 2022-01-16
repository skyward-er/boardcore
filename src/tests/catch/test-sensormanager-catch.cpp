/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#ifdef STANDALONE_CATCH1_TEST
#include "catch-tests-entry.cpp"
#endif

#include <utils/testutils/TestSensor.h>

#include <catch2/catch.hpp>
#include <iostream>

#define private public
#define protected public

#include <sensors/SensorInfo.h>
#include <sensors/SensorManager.h>

namespace Boardcore
{

static const uint8_t FIRST_TASK_ID = 7;  // used to test IDs assignment to tasks

class FailingSensor : public Sensor<TestData>
{
    bool init() { return true; }

    bool selfTest() { return false; }  // always fail self-test

    TestData sampleImpl() { return TestData{}; }
};

class SensorManagerFixture
{
public:
    SensorManagerFixture()
    {
        // cppcheck-suppress noCopyConstructor
        // cppcheck-suppress noOperatorEq
        scheduler = new TaskScheduler();
        scheduler->add([]() { std::cout << "Task Callback!" << std::endl; },
                       2000,  // inserst a test function in the scheduler
                       FIRST_TASK_ID);

        sensorManager = new SensorManager({{&s1, s1_info},
                                           {&s2, s2_info},
                                           {&s3, s3_info},
                                           {&s4, s4_info},
                                           {&s5, s5_info}},
                                          scheduler);

        sampler1 = sensorManager->samplersMap[&s1];
        sampler2 = sensorManager->samplersMap[&s2];
        sampler3 = sensorManager->samplersMap[&s3];
        sampler4 = sensorManager->samplersMap[&s4];
        sampler5 = sensorManager->samplersMap[&s5];
    }

    ~SensorManagerFixture()
    {
        sensorManager->stop();

        delete sensorManager;
    }

private:
    TaskScheduler* scheduler;

    SensorManager* sensorManager;

    SensorSampler* sampler1;
    SensorSampler* sampler2;
    SensorSampler* sampler3;
    SensorSampler* sampler4;
    SensorSampler* sampler5;

    TestSensor s1;
    SensorInfo s1_info{
        /*ID=*/"s1",
        /*Period=*/1000,
        /*Callback=*/[]() { std::cout << "Callback 1!" << std::endl; },
        /*Enabled=*/true};

    TestSensor s2;
    SensorInfo s2_info{
        /*ID=*/"s2",
        /*Period=*/1000,
        /*Callback=*/[]() { std::cout << "Callback 2!" << std::endl; },
        /*Enabled=*/false};

    TestSensor s3;
    SensorInfo s3_info{
        /*ID=*/"s3",
        /*Period=*/500,
        /*Callback=*/[]() { std::cout << "Callback 3!" << std::endl; },
        /*Enabled=*/true};

    TestSensor s4;
    SensorInfo s4_info{
        /*ID=*/"s4",
        /*Period=*/1000,
        /*Callback=*/[]() { std::cout << "Callback 4!" << std::endl; },
        /*Enabled=*/true};

    // always failing self-test
    FailingSensor s5;
    SensorInfo s5_info{
        /*ID=*/"s5",
        /*Period=*/2000,
        /*Callback=*/[]() { std::cout << "Callback 5!" << std::endl; },
        /*Enabled=*/true};
};

bool operator==(const SensorInfo& lhs, const SensorInfo& rhs)
{
    return lhs.id == rhs.id && lhs.period == rhs.period &&
           lhs.callback.target_type() == rhs.callback.target_type() &&
           lhs.callback.target<void()>() == rhs.callback.target<void()>() &&
           lhs.isEnabled == rhs.isEnabled;
}

bool operator==(const SensorSampler& lhs, const SensorSampler& rhs)
{
    return lhs.id == rhs.id && lhs.period == rhs.period &&
           lhs.sensors.size() == rhs.sensors.size();
}

}  // namespace Boardcore

using namespace Boardcore;

TEST_CASE_METHOD(SensorManagerFixture,
                 "Samplers IDs should incrementally start from FIRST_TASK_ID")
{
    sensorManager->start();

    vector<TaskStatResult> tasksStats = scheduler->getTaskStats();

    std::cout << "Tasks number : " << tasksStats.size() << std::endl;

    // Sampler with lower period are inserted in the TaskScheduler
    // before higher period ones
    // =>
    //    Task id 8  : sampler at 1000 ms (1 Hz)
    //    Task id 9  : sampler at 500 ms  (2 Hz) ---> first to be added to the
    //                                                scheduler
    REQUIRE(tasksStats[0].id == FIRST_TASK_ID);
    REQUIRE(tasksStats[1].id == static_cast<uint8_t>(FIRST_TASK_ID + 2));
    REQUIRE(tasksStats[2].id == static_cast<uint8_t>(FIRST_TASK_ID + 1));
    REQUIRE(tasksStats[3].id == static_cast<uint8_t>(FIRST_TASK_ID + 3));
    REQUIRE(tasksStats[4].id == static_cast<uint8_t>(FIRST_TASK_ID + 4));
}

TEST_CASE_METHOD(SensorManagerFixture,
                 "Sensors are correctly added to the samplers")
{
    // check that 3 samplers exist (1 hz, 2 hz and 0.5 Hz)
    REQUIRE(sensorManager->samplers.size() == 3);

    // samplers are sorted by period, in decreasing order!

    // check that s1, s2 and s3 are assigned to correct samplers
    REQUIRE(sampler1 == sensorManager->samplers[1]);
    REQUIRE(*sampler1 == *(sensorManager->samplers[1]));
    // s1 and s2 are assigned to same sampler
    REQUIRE(sampler1 == sampler2);
    REQUIRE(*sampler1 == *sampler2);
    // s3 assigned to another sampler
    REQUIRE(sampler3 == sensorManager->samplers[0]);
    REQUIRE(*sampler3 == *(sensorManager->samplers[0]));
    // s4 assigned to another sampler
    REQUIRE(sampler4 == sensorManager->samplers[2]);
    REQUIRE(*sampler4 == *(sensorManager->samplers[2]));
    // s5 assigned to the last sampler
    REQUIRE(sampler5 == sensorManager->samplers[3]);
    REQUIRE(*sampler5 == *(sensorManager->samplers[3]));
}

TEST_CASE_METHOD(SensorManagerFixture,
                 "Sensors are correctly coupled with their info and failing "
                 "sensors are automatically disabled")
{
    SensorInfo info1 = sampler1->getSensorInfo(&s1);
    SensorInfo info2 = sampler2->getSensorInfo(&s2);
    SensorInfo info3 = sampler3->getSensorInfo(&s3);
    SensorInfo info4 = sampler4->getSensorInfo(&s4);
    SensorInfo info5 = sampler5->getSensorInfo(&s5);

    // correctly initialized sensors
    REQUIRE(s1_info == info1);
    REQUIRE(s2_info == info2);
    REQUIRE(s3_info == info3);
    REQUIRE(s4_info == info4);

    REQUIRE(
        !(s5_info ==
          info5));  // it fails, so isEnabled is set to false instead of true
    REQUIRE(s5_info.id == info5.id);
    REQUIRE(s5_info.period == info5.period);
    REQUIRE(info5.isEnabled ==
            false);  // disabled even if it was created as enabled
    REQUIRE(info5.isInitialized == false);  // always fails the initialization
}

TEST_CASE_METHOD(SensorManagerFixture,
                 "Samplers have the correct number of sensors")
{
    // sampler at 500 ms (2 Hz) has 1 sensor
    // sampler at 1000 ms (1 Hz) has 3 sensors
    // sampler at 2000 ms (2 Hz) has 1 sensor
    for (auto s : sensorManager->samplers)
    {
        if (s->getSamplingPeriod() == 1000)
        {
            REQUIRE(s->getNumSensors() == 3);
        }
        else if (s->getSamplingPeriod() == 500)
        {
            REQUIRE(s->getNumSensors() == 1);
        }
        else if (s->getSamplingPeriod() == 2000)
        {
            REQUIRE(s->getNumSensors() == 1);
        }
        else
        {
            FAIL(
                "Can't exist a sampler with period different from 500, 1000 or "
                "2000 ms");  // no sampler with a different period exist
        }
    }
}

TEST_CASE_METHOD(SensorManagerFixture, "Enable/disable sensors at runtime")
{
    sensorManager->start();

    sensorManager->enableSensor(&s2);
    sensorManager->disableSensor(&s4);

    REQUIRE(sensorManager->getSensorInfo(&s2).isEnabled == true);
    REQUIRE(sensorManager->getSensorInfo(&s4).isEnabled == false);

    sensorManager->disableSensor(&s2);
    sensorManager->enableSensor(&s4);

    REQUIRE(sensorManager->getSensorInfo(&s2).isEnabled == false);
    REQUIRE(sensorManager->getSensorInfo(&s4).isEnabled == true);
}

TEST_CASE_METHOD(SensorManagerFixture, "Enable/disable all sensors at runtime")
{
    sensorManager->start();

    sensorManager->enableAllSensors();

    REQUIRE(sensorManager->getSensorInfo(&s1).isEnabled == true);
    REQUIRE(sensorManager->getSensorInfo(&s2).isEnabled == true);
    REQUIRE(sensorManager->getSensorInfo(&s3).isEnabled == true);
    REQUIRE(sensorManager->getSensorInfo(&s4).isEnabled == true);
    REQUIRE(sensorManager->getSensorInfo(&s5).isEnabled == true);

    sensorManager->disableAllSensors();

    REQUIRE(sensorManager->getSensorInfo(&s1).isEnabled == false);
    REQUIRE(sensorManager->getSensorInfo(&s2).isEnabled == false);
    REQUIRE(sensorManager->getSensorInfo(&s3).isEnabled == false);
    REQUIRE(sensorManager->getSensorInfo(&s4).isEnabled == false);
    REQUIRE(sensorManager->getSensorInfo(&s5).isEnabled == false);

    sensorManager->enableAllSensors();

    REQUIRE(sensorManager->getSensorInfo(&s1).isEnabled == true);
    REQUIRE(sensorManager->getSensorInfo(&s2).isEnabled == true);
    REQUIRE(sensorManager->getSensorInfo(&s3).isEnabled == true);
    REQUIRE(sensorManager->getSensorInfo(&s4).isEnabled == true);
    REQUIRE(sensorManager->getSensorInfo(&s5).isEnabled == true);
}

TEST_CASE_METHOD(SensorManagerFixture,
                 "Try to get info about a non-existing sensor")
{
    TestSensor invalidSensor;
    SensorInfo invalidInfo = sensorManager->getSensorInfo(&invalidSensor);
    REQUIRE(invalidInfo == SensorInfo{});

    SensorInfo validInfo = sensorManager->getSensorInfo(&s2);
    REQUIRE(validInfo == s2_info);
}
