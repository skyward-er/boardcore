/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Conterio
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

#ifdef STANDALONE_CATCH1_TEST
#include "catch-tests-entry.cpp"
#endif

#define private public
#define protected public

#include "sensors/SensorInfo.h"
#include "sensors/SensorManager.h"
#include "utils/testutils/TestSensor.h"
#include "utils/testutils/catch.hpp"

static const uint8_t FIRST_TASK_ID = 7;  // used to test IDs assignment to tasks

class SensorManagerFixture
{
public:
    SensorManagerFixture()
    {
        scheduler = new TaskScheduler();
        scheduler->add([]() { std::cout << "Task Callback!" << endl; },
                       2000,  // inserst a test function in the scheduler
                       FIRST_TASK_ID);

        sensor_manager = new SensorManager(
            scheduler,
            {{&s1, s1_info}, {&s2, s2_info}, {&s3, s3_info}, {&s4, s4_info}});

        sampler1 = sensor_manager->samplers_map[&s1];
        sampler2 = sensor_manager->samplers_map[&s2];
        sampler3 = sensor_manager->samplers_map[&s3];
        sampler4 = sensor_manager->samplers_map[&s4];
    }

    ~SensorManagerFixture()
    {
        sensor_manager->stop();

        delete sensor_manager;
    }

private:
    TaskScheduler* scheduler;

    SensorManager* sensor_manager;

    SensorSampler* sampler1;
    SensorSampler* sampler2;
    SensorSampler* sampler3;
    SensorSampler* sampler4;

    TestSensor s1;
    SensorInfo s1_info{
        /*Freq=*/1,
        /*Callback=*/[]() { std::cout << "Callback 1!" << endl; },
        /*DMA=*/false,
        /*Enabled=*/true};

    TestSensor s2;
    SensorInfo s2_info{
        /*Freq=*/1,
        /*Callback=*/[]() { std::cout << "Callback 2!" << endl; },
        /*DMA=*/false,
        /*Enabled=*/false};

    TestSensor s3;
    SensorInfo s3_info{
        /*Freq=*/2,
        /*Callback=*/[]() { std::cout << "Callback 3!" << endl; },
        /*DMA=*/false,
        /*Enabled=*/true};

    // same freq as s1 and s2 but uses DMA
    TestSensor s4;
    SensorInfo s4_info{
        /*Freq=*/1,
        /*Callback=*/[]() { std::cout << "Callback 4!" << endl; },
        /*DMA=*/true,
        /*Enabled=*/true};
};

bool operator==(const SensorInfo& lhs, const SensorInfo& rhs)
{
    return lhs.freq == rhs.freq &&
           lhs.callback.target_type() == rhs.callback.target_type() &&
           lhs.callback.target<void()>() == rhs.callback.target<void()>() &&
           lhs.is_dma == rhs.is_dma && lhs.is_enabled == rhs.is_enabled;
}

bool operator==(const SensorSampler& lhs, const SensorSampler& rhs)
{
    return lhs.id == rhs.id && lhs.freq == rhs.freq &&
           lhs.is_dma == rhs.is_dma &&
           lhs.sensors_map.size() == rhs.sensors_map.size();
}

TEST_CASE_METHOD(SensorManagerFixture,
                 "Samplers IDs should incrementally start from FIRST_TASK_ID")
{
    sensor_manager->start();

    std::vector<TaskStatResult> tasks_stats = scheduler->getTaskStats();

    std::cout << "Tasks number : " << tasks_stats.size() << endl;

    std::cout << "Task IDs : " << endl;
    for (auto stats : tasks_stats)
    {
        printf("%d \n", stats.id);
    }

    REQUIRE(tasks_stats[0].id == FIRST_TASK_ID);
    REQUIRE(tasks_stats[1].id == static_cast<uint8_t>(FIRST_TASK_ID + 1));
    REQUIRE(tasks_stats[2].id == static_cast<uint8_t>(FIRST_TASK_ID + 2));
    REQUIRE(tasks_stats[3].id == static_cast<uint8_t>(FIRST_TASK_ID + 3));
}

TEST_CASE_METHOD(SensorManagerFixture,
                 "Sensors are correctly added to the samplers")
{
    // check that 3 samplers exist (1 hz, 2 hz and 1 hz with DMA)
    REQUIRE(sensor_manager->samplers.size() == 3);

    // check that s1, s2 and s3 are assigned to correct samplers
    REQUIRE(sampler1 == sensor_manager->samplers[0]);
    REQUIRE(*sampler1 == *(sensor_manager->samplers[0]));
    // s1 and s2 are assigned to same sampler
    REQUIRE(sampler1 == sampler2);
    REQUIRE(*sampler1 == *sampler2);
    // s3 assigned to the other sampler
    REQUIRE(sampler3 == sensor_manager->samplers[1]);
    REQUIRE(*sampler3 == *(sensor_manager->samplers[1]));
    // s4 assigned to the last sampler
    REQUIRE(sampler4 == sensor_manager->samplers[2]);
    REQUIRE(*sampler4 == *(sensor_manager->samplers[2]));
}

TEST_CASE_METHOD(SensorManagerFixture,
                 "Sensors are correctly coupled with their info")
{
    SensorInfo info1 = sampler1->getSensorInfo(&s1);
    SensorInfo info2 = sampler2->getSensorInfo(&s2);
    SensorInfo info3 = sampler3->getSensorInfo(&s3);
    SensorInfo info4 = sampler4->getSensorInfo(&s4);

    REQUIRE(s1_info == info1);
    REQUIRE(s2_info == info2);
    REQUIRE(s3_info == info3);
    REQUIRE(s4_info == info4);
}

TEST_CASE_METHOD(SensorManagerFixture,
                 "Samplers have the correct number of sensors")
{
    // check that sampler at 1 Hz has 2 sensors
    // sampler at 2 Hz has 1 sensor
    // sampler at 1 Hz with DMA has 1 sensor
    for (auto s : sensor_manager->samplers)
    {
        if (s->getFrequency() == 1 && s->isDMA() == false)
        {
            REQUIRE(s->getNumSensors() == 2);
        }
        else if (s->getFrequency() == 2)
        {
            REQUIRE(s->getNumSensors() == 1);
        }
        else if (s->getFrequency() == 1 && s->isDMA() == true)
        {
            REQUIRE(s->getNumSensors() == 1);
        }
        else
        {
            FAIL(
                "Can't exist a sampler with frequency different from 1 or 2 "
                "Hz");  // no sampler with a different frequency exist
        }
    }
}

TEST_CASE_METHOD(SensorManagerFixture,
                 "Sensors can be enabled/disabled at runtime")
{
    sensor_manager->start();

    Thread::sleep(2000);

    sensor_manager->enableSensor(&s2);
    sensor_manager->disableSensor(&s4);

    // Thread::sleep(300);
    REQUIRE(sensor_manager->getSensorInfo(&s2).is_enabled == true);
    REQUIRE(sensor_manager->getSensorInfo(&s4).is_enabled == false);

    Thread::sleep(2000);

    sensor_manager->disableSensor(&s2);
    sensor_manager->enableSensor(&s4);

    // Thread::sleep(300);
    REQUIRE(sensor_manager->getSensorInfo(&s2).is_enabled == false);
    REQUIRE(sensor_manager->getSensorInfo(&s4).is_enabled == true);

    Thread::sleep(1000);
}