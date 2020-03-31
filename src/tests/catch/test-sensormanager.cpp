/* Copyright (c) 2018-2020 Skyward Experimental Rocketry
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

#include "utils/testutils/catch.hpp"

#include "utils/testutils/TestSensor.h"
#include "sensors/SensorManager.h"

TestSensor* sensor1 = new TestSensor();
TestSensor* sensor2 = new TestSensor();
TestSensor* sensor3 = new TestSensor();


TEST_CASE("check that sensors are correctly added to the samplers") {

    SensorManager sensor_manager;

    sensor_manager.addSensor(sensor1, 1, std::bind([&]() {})); // 1 Hz
    sensor_manager.addSensor(sensor2, 1, std::bind([&]() {})); // 1 Hz
    sensor_manager.addSensor(sensor3, 2, std::bind([&]() {})); // 2 Hz

    // check that 2 samplers exist (1 hz and 2 hz)
    REQUIRE(sensor_manager.getSamplers().size() == 2);
    
    // check that sampler at 1 Hz has 2 sensors, sampler at 2 Hz has 1 sensor
    for(auto s : sensor_manager.getSamplers()) {
        if (s.sampler->getFrequency() == 1) {
            REQUIRE(s.sampler->getNumSensors() == 2);
        }
        else if (s.sampler->getFrequency() == 2) {
            REQUIRE(s.sampler->getNumSensors() == 1);
        }
        else {
            FAIL("Can't exist a sampler with frequency different from 1 or 2 Hz"); // no sampler with a different frequency exist
        }
    }
}