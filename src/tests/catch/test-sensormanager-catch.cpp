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

#include <utils/TestUtils/TestSensor.h>

#include <catch2/catch.hpp>
#include <iostream>

#define private public
#define protected public

#include <sensors/SensorInfo.h>
#include <sensors/SensorConfig.h>
#include <sensors/SensorGroup.h>
#include <sensors/SensorManager.h>

using namespace Boardcore;
using namespace Boardcore::Units::Frequency;
using namespace std::chrono_literals;

static const size_t FIRST_TASK_ID = 1;  // used to test IDs assignment to tasks

class FailingSensorCatch : public Sensor<TestData>
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
        schedulerGroup0 = std::make_unique<TaskScheduler>();
        // inserst a test function in the scheduler
        schedulerGroup0->addTask(
            []() { std::cout << "Task Callback!" << std::endl; }, 2000);

        SensorManager::SchedulerMap_t schedMap{{0, schedulerGroup0.get()}};
        SensorManager::SensorMap_t sensMap{{&s1, s1_conf},
                                           {&s2, s2_conf},
                                           {&s3, s3_conf},
                                           {&s4, s4_conf},
                                           {&s5, s5_conf}};

        sensorManager = std::make_unique<SensorManager>(sensMap, schedMap);

        samplerSensor1 =
            sensorManager->groups[s1_conf.groupID]->samplersMap[&s1].get();
        samplerSensor2 =
            sensorManager->groups[s2_conf.groupID]->samplersMap[&s2].get();
        samplerSensor3 =
            sensorManager->groups[s3_conf.groupID]->samplersMap[&s3].get();
        samplerSensor4 =
            sensorManager->groups[s4_conf.groupID]->samplersMap[&s4].get();
        samplerSensor5 =
            sensorManager->groups[s5_conf.groupID]->samplersMap[&s5].get();
    }

    ~SensorManagerFixture() { sensorManager->stop(); }

    SensorManagerFixture& operator=(SensorManagerFixture const&) = delete;
    SensorManagerFixture(const SensorManagerFixture& p)          = delete;

    /**
     * @brief Utility used to compare the 2 structs. Return true
     * if the passed structs are similar (they refer to the same
     * sensor).
     */
    static bool compareInfoAndConfig(const SensorInfo& info,
                                     const SensorConfig& conf)
    {
        return info.groupID == conf.groupID && info.id == conf.id &&
               info.period == conf.period;
    }

private:
    std::unique_ptr<TaskScheduler> schedulerGroup0;  // Scheduler for group 0

    std::unique_ptr<SensorManager> sensorManager;

    SensorSampler* samplerSensor1;
    SensorSampler* samplerSensor2;
    SensorSampler* samplerSensor3;
    SensorSampler* samplerSensor4;
    SensorSampler* samplerSensor5;

    TestSensor s1;
    SensorConfig s1_conf{/*ID=*/"s1",
                         /*Period=*/1000,
                         /*Callback=*/[]()
                         { std::cout << "Callback 1!" << std::endl; },
                         /*Enabled=*/true,
                         /*GroupId=*/0};

    TestSensor s2;
    SensorConfig s2_conf{/*ID=*/"s2",
                         /*Period=*/1000ms,
                         /*Callback=*/[]()
                         { std::cout << "Callback 2!" << std::endl; },
                         /*Enabled=*/false,
                         /*GroupId=*/0};

    TestSensor s3;
    SensorConfig s3_conf{/*ID=*/"s3",
                         /*Period=*/500,
                         /*Callback=*/[]()
                         { std::cout << "Callback 3!" << std::endl; },
                         /*Enabled=*/true,
                         /*GroupId=*/1};

    TestSensor s4;
    SensorConfig s4_conf{/*ID=*/"s4",
                         /*Period=*/1_hz,
                         /*Callback=*/[]()
                         { std::cout << "Callback 4!" << std::endl; },
                         /*Enabled=*/true,
                         /*GroupId=*/1};

    // always failing self-test
    FailingSensorCatch s5;
    SensorConfig s5_conf{/*ID=*/"s5",
                         /*Period=*/2000,
                         /*Callback=*/[]()
                         { std::cout << "Callback 5!" << std::endl; },
                         /*Enabled=*/true,
                         /*GroupId=*/1};
};

TEST_CASE_METHOD(SensorManagerFixture,
                 "Samplers IDs should incrementally start from FIRST_TASK_ID")
{
    sensorManager->start();

    vector<TaskStatsResult> tasksStats = schedulerGroup0->getTaskStats();

    INFO("Tasks number : " << tasksStats.size());

    // Sampler with lower period are inserted in the TaskScheduler
    // before higher period ones
    // =>
    //    Task id 8  : sampler at 1000 ms (1 Hz)
    //    Task id 9  : sampler at 500 ms  (2 Hz) ---> first to be added to the
    //                                                scheduler
    REQUIRE(tasksStats[0].id == FIRST_TASK_ID);
    REQUIRE(tasksStats[1].id == static_cast<uint8_t>(FIRST_TASK_ID + 1));
    REQUIRE(tasksStats[2].id == static_cast<uint8_t>(FIRST_TASK_ID + 2));
    REQUIRE(tasksStats[3].id == static_cast<uint8_t>(FIRST_TASK_ID + 3));
}

TEST_CASE_METHOD(SensorManagerFixture,
                 "Sensors are correctly added to the samplers")
{
    // Check that only 1 sampler exists in group0
    REQUIRE(sensorManager->groups[0]->samplers.size() == 1);
    // Check that 3 samplers exist in group1
    REQUIRE(sensorManager->groups[1]->samplers.size() == 3);

    // Sensors s1 and s2 are assigned to group0

    // Check that s1 and s2 are assigned to the correct sampler
    REQUIRE(samplerSensor1 == sensorManager->groups[0]->samplers[0].get());
    REQUIRE(*samplerSensor1 == *(sensorManager->groups[0]->samplers[0].get()));
    // s1 and s2 are assigned to same sampler
    REQUIRE(samplerSensor1 == samplerSensor2);
    REQUIRE(*samplerSensor1 == *samplerSensor2);

    // Sensors s3, s4 and s5 are assigned to group1
    // - s3 -> 2 Hz
    // - s4 -> 1 Hz
    // - s5 -> 0.5 Hz
    // Samplers are sorted by period, in decreasing order!

    // Check that s3 is assigned to the first sampler
    REQUIRE(samplerSensor3 == sensorManager->groups[1]->samplers[0].get());
    REQUIRE(*samplerSensor3 == *(sensorManager->groups[1]->samplers[0].get()));
    // Check that s4 is assigned to the second sampler
    REQUIRE(samplerSensor4 == sensorManager->groups[1]->samplers[1].get());
    REQUIRE(*samplerSensor4 == *(sensorManager->groups[1]->samplers[1].get()));
    // Check that s5 is assigned to the last sampler
    REQUIRE(samplerSensor5 == sensorManager->groups[1]->samplers[2].get());
    REQUIRE(*samplerSensor5 == *(sensorManager->groups[1]->samplers[2].get()));
}

TEST_CASE_METHOD(SensorManagerFixture,
                 "Sensors are correctly coupled with their info and failing "
                 "sensors are automatically disabled")
{
    SensorInfo info1 = samplerSensor1->getSensorInfo(&s1);
    SensorInfo info2 = samplerSensor2->getSensorInfo(&s2);
    SensorInfo info3 = samplerSensor3->getSensorInfo(&s3);
    SensorInfo info4 = samplerSensor4->getSensorInfo(&s4);
    SensorInfo info5 = samplerSensor5->getSensorInfo(&s5);

    // Correctly initialized sensors
    REQUIRE(compareInfoAndConfig(info1, s1_conf));
    REQUIRE(compareInfoAndConfig(info2, s2_conf));
    REQUIRE(compareInfoAndConfig(info3, s3_conf));
    REQUIRE(compareInfoAndConfig(info4, s4_conf));

    // Sensor 5 should have failed
    REQUIRE(!compareInfoAndConfig(info5, s5_conf));
    REQUIRE(s5_conf.id == info5.id);
    REQUIRE(s5_conf.period == info5.period);
    REQUIRE(info5.status ==
            SensorStatus::NOT_INIT);  // s5 always fails the initialization
}

TEST_CASE_METHOD(
    SensorManagerFixture,
    "Samplers have the correct number of sensors and the correct periods")
{
    for (auto& group : sensorManager->groups)
    {
        if (group.first == 0)
        {
            // Group0: 1 sampler (1 Hz) with 2 sensors
            REQUIRE(group.second->samplers.size() ==
                    1);  // It must contain only 1 sampler

            auto& sampler = group.second->samplers[0];

            REQUIRE(sampler->getSamplingPeriod() ==
                    std::chrono::milliseconds(1000));
            REQUIRE(sampler->getNumSensors() == 2);
        }
        else if (group.first == 1)  // Group1
        {
            // Group1: 3 samplers (0.5 Hz, 1 Hz, 2 Hz) each one with 1 sensor
            REQUIRE(group.second->samplers.size() ==
                    3);  // It must have 3 samplers

            // NOTE: samplers should be ordered with increasing period (lower
            // frequency)
            REQUIRE(group.second->samplers[0]->getSamplingPeriod() ==
                    std::chrono::milliseconds(500));
            REQUIRE(group.second->samplers[0]->getNumSensors() == 1);

            REQUIRE(group.second->samplers[1]->getSamplingPeriod() ==
                    std::chrono::seconds(1));
            REQUIRE(group.second->samplers[1]->getNumSensors() == 1);

            REQUIRE(group.second->samplers[2]->getSamplingPeriod() ==
                    std::chrono::seconds(2));
            REQUIRE(group.second->samplers[2]->getNumSensors() == 1);
        }
        else
        {
            FAIL(
                "There shouldn't exist a group with id different from 0 and 1");
        }
    }
}

TEST_CASE_METHOD(SensorManagerFixture, "Enable/disable sensors at runtime")
{
    sensorManager->start();

    sensorManager->enableSensor(&s2);
    sensorManager->disableSensor(&s4);

    REQUIRE(sensorManager->getSensorInfo(&s2).status == SensorStatus::ENABLED);
    REQUIRE(sensorManager->getSensorInfo(&s4).status == SensorStatus::DISABLED);

    sensorManager->disableSensor(&s2);
    sensorManager->enableSensor(&s4);

    REQUIRE(sensorManager->getSensorInfo(&s2).status == SensorStatus::DISABLED);
    REQUIRE(sensorManager->getSensorInfo(&s4).status == SensorStatus::ENABLED);
}

TEST_CASE_METHOD(SensorManagerFixture, "Enable/disable all sensors at runtime")
{
    sensorManager->start();

    sensorManager->enableAllSensors();

    REQUIRE(sensorManager->getSensorInfo(&s1).status == SensorStatus::ENABLED);
    REQUIRE(sensorManager->getSensorInfo(&s2).status == SensorStatus::ENABLED);
    REQUIRE(sensorManager->getSensorInfo(&s3).status == SensorStatus::ENABLED);
    REQUIRE(sensorManager->getSensorInfo(&s4).status == SensorStatus::ENABLED);
    REQUIRE(sensorManager->getSensorInfo(&s5).status ==
            SensorStatus::NOT_INIT);  // s5 always fails init, the state mustn't
                                      // change

    sensorManager->disableAllSensors();

    REQUIRE(sensorManager->getSensorInfo(&s1).status == SensorStatus::DISABLED);
    REQUIRE(sensorManager->getSensorInfo(&s2).status == SensorStatus::DISABLED);
    REQUIRE(sensorManager->getSensorInfo(&s3).status == SensorStatus::DISABLED);
    REQUIRE(sensorManager->getSensorInfo(&s4).status == SensorStatus::DISABLED);
    REQUIRE(sensorManager->getSensorInfo(&s5).status ==
            SensorStatus::NOT_INIT);  // s5 always fails init, the state mustn't
                                      // change

    sensorManager->enableAllSensors();

    REQUIRE(sensorManager->getSensorInfo(&s1).status == SensorStatus::ENABLED);
    REQUIRE(sensorManager->getSensorInfo(&s2).status == SensorStatus::ENABLED);
    REQUIRE(sensorManager->getSensorInfo(&s3).status == SensorStatus::ENABLED);
    REQUIRE(sensorManager->getSensorInfo(&s4).status == SensorStatus::ENABLED);
    REQUIRE(sensorManager->getSensorInfo(&s5).status ==
            SensorStatus::NOT_INIT);  // s5 always fails init, the state mustn't
                                      // change
}

TEST_CASE_METHOD(SensorManagerFixture,
                 "Try to get info about a non-existing sensor")
{
    TestSensor invalidSensor;
    SensorInfo invalidInfo = sensorManager->getSensorInfo(&invalidSensor);
    REQUIRE(invalidInfo.status == SensorStatus::NOT_INIT);
}
