/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Nicolò Caruso
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
#include <stdint.h>
// #include <utils/Registry/RegistryFrontend.h>
/*! TODO: Re-add it when the middleware is integrated again */
// #include <utils/Registry/RegistryMiddleware.h>

#include <catch2/catch.hpp>
#include <utils/Registry/RegistryFrontend.cpp>
#include <utils/Registry/RegistrySerializer.cpp>
/*! TODO: Re-add it when the middleware is integrated again */
// #include <utils/Registry/RegistryMiddleware.cpp>

using namespace Boardcore;

/** Magic numbers for testing the set/get for each data type, they do not
 * reflect the reals configuration ids and values in use! */
static constexpr uint8_t TEST_VALUE_UINT8                = 20;
static constexpr uint32_t TEST_VALUE_UINT32              = 30;
static constexpr float TEST_VALUE_FLOAT                  = 1.45;
static constexpr float TEST_VALUE_LATITUDE               = 45.50109;
static constexpr float TEST_VALUE_LONGITUDE              = 9.15633;
static constexpr uint32_t COORDINATE_ID                  = 8;
static constexpr uint32_t DEPLOYMENT_ALTITUDE_ID         = 50;
static constexpr uint32_t TARGET_COORDINATES_ID          = 1;
static constexpr uint32_t VENTING_VALVE_ATOMIC_TIMING_ID = 49;
static constexpr uint32_t ALGORITHM_ID                   = 128;
static constexpr uint32_t IGNITION_TIME_ID               = 3;
static constexpr uint32_t FLOAT_VALUE_ID                 = 13;

TEST_CASE(
    "RegistryFrontend test - Set and get type Unsafe configuration entries")
{
    RegistryFrontend registry;
    float floatValue;
    uint32_t uint32Value;
    uint8_t uint8Value;
    Coordinates coordinatesValue(TEST_VALUE_LATITUDE, TEST_VALUE_LONGITUDE);
    /*! Check that the registry is first empty */
    REQUIRE(registry.isConfigurationEmpty());
    /*! Checks that there are effectively non-initialized entry configurations
     */
    REQUIRE_FALSE(registry.getConfigurationUnsafe(
        static_cast<uint32_t>(DEPLOYMENT_ALTITUDE_ID), floatValue));
    REQUIRE_FALSE(registry.getConfigurationUnsafe(
        static_cast<uint32_t>(TARGET_COORDINATES_ID), coordinatesValue));
    REQUIRE_FALSE(registry.getConfigurationUnsafe(
        static_cast<uint32_t>(VENTING_VALVE_ATOMIC_TIMING_ID), uint32Value));
    REQUIRE_FALSE(registry.getConfigurationUnsafe(
        static_cast<uint32_t>(ALGORITHM_ID), uint8Value));
    /*! Check set configuration results in right get */
    REQUIRE(registry.setConfigurationUnsafe(static_cast<uint32_t>(ALGORITHM_ID),
                                            TEST_VALUE_UINT8));
    uint8Value = 0;
    REQUIRE(registry.getConfigurationUnsafe(ALGORITHM_ID, uint8Value));
    REQUIRE(uint8Value == TEST_VALUE_UINT8);
    uint32Value = 0;
    REQUIRE(registry.setConfigurationUnsafe(COORDINATE_ID, TEST_VALUE_UINT32));
    REQUIRE(registry.getConfigurationUnsafe(COORDINATE_ID, uint32Value));
    REQUIRE(uint32Value == TEST_VALUE_UINT32);
    /*! Checks that get configuration is false if the type is incorrect w.r.t.
     * the type of the set type */
    REQUIRE_FALSE(registry.getConfigurationUnsafe(COORDINATE_ID, floatValue));
}

TEST_CASE("RegistryFrontend test - Arm/Disarm test")
{
    RegistryFrontend registry;
    uint8_t uint8Value;
    Coordinates coordinatesValue(TEST_VALUE_LATITUDE, TEST_VALUE_LONGITUDE),
        coordinateGet;
    REQUIRE(registry.setConfigurationUnsafe(COORDINATE_ID, coordinatesValue));
    registry.arm();
    /*! If the registry is "armed" no set are allowed but gets are */
    REQUIRE_FALSE(
        registry.setConfigurationUnsafe(ALGORITHM_ID, TEST_VALUE_UINT8));
    REQUIRE_FALSE(registry.getConfigurationUnsafe(ALGORITHM_ID, uint8Value));
    REQUIRE(registry.getConfigurationUnsafe(COORDINATE_ID, coordinateGet));
    REQUIRE(coordinateGet.latitude == coordinatesValue.latitude);
    REQUIRE(coordinateGet.longitude == coordinatesValue.longitude);

    /*! DISARM AND SET NEW ENTRIES */

    registry.disarm();
    REQUIRE(registry.setConfigurationUnsafe(ALGORITHM_ID, TEST_VALUE_UINT8));
    REQUIRE(registry.getConfigurationUnsafe(ALGORITHM_ID, uint8Value));
    REQUIRE(uint8Value == TEST_VALUE_UINT8);
}

TEST_CASE("RegistryFrontend test - serialization/deserialization test")
{
    RegistryFrontend registry;
    Coordinates coordinatesValue(TEST_VALUE_LATITUDE, TEST_VALUE_LONGITUDE),
        coordinateGet(0, 0);
    uint32_t valueInt = 0;
    float valueFloat  = 0;

    registry.clear();
    /*! FIRST SET OF THE CONFIGURATION */
    REQUIRE(registry.setConfigurationUnsafe(COORDINATE_ID, coordinatesValue));
    registry.saveConfiguration();

    /*! LOAD AND CHECK CONFIGURATION */
    registry.saveConfiguration();
    REQUIRE(registry.getConfigurationUnsafe(COORDINATE_ID, coordinateGet));
    REQUIRE(registry.loadConfiguration());
    REQUIRE(registry.loadConfiguration());
    REQUIRE(registry.getConfigurationUnsafe(COORDINATE_ID, coordinateGet));
    REQUIRE(coordinateGet.latitude == coordinatesValue.latitude);
    REQUIRE(coordinateGet.longitude == coordinatesValue.longitude);

    /*! SET OTHER DATA TYPES CONFIGURATIONS ENTRIES*/

    REQUIRE(registry.setConfigurationUnsafe((VENTING_VALVE_ATOMIC_TIMING_ID),
                                            TEST_VALUE_UINT32));
    registry.saveConfiguration();

    valueInt = 0;
    registry.saveConfiguration();
    REQUIRE(registry.getConfigurationUnsafe(VENTING_VALVE_ATOMIC_TIMING_ID,
                                            valueInt));
    REQUIRE(valueInt == TEST_VALUE_UINT32);
    registry.saveConfiguration();

    REQUIRE(registry.setConfigurationUnsafe(FLOAT_VALUE_ID, TEST_VALUE_FLOAT));
    registry.saveConfiguration();
    valueFloat = 0;
    registry.saveConfiguration();

    REQUIRE(registry.getConfigurationUnsafe(VENTING_VALVE_ATOMIC_TIMING_ID,
                                            valueInt));
    REQUIRE(valueInt == TEST_VALUE_UINT32);
    REQUIRE(registry.getConfigurationUnsafe(FLOAT_VALUE_ID, valueFloat));
    REQUIRE(valueFloat == TEST_VALUE_FLOAT);

    /*! SAVE AGAIN WITH ALL TYPES INSIDE CONFIGURATION */

    REQUIRE(registry.getConfigurationUnsafe(COORDINATE_ID, coordinateGet));

    registry.saveConfiguration();
    registry.saveConfiguration();
    registry.saveConfiguration();
    registry.saveConfiguration();
    registry.saveConfiguration();
    registry.saveConfiguration();
    registry.loadConfiguration();

    REQUIRE(registry.getConfigurationUnsafe(COORDINATE_ID, coordinateGet));

    registry.saveConfiguration();
    REQUIRE(registry.loadConfiguration());
    registry.saveConfiguration();
    REQUIRE(registry.loadConfiguration());
    registry.saveConfiguration();
    REQUIRE(registry.loadConfiguration());
    REQUIRE(registry.loadConfiguration());
    REQUIRE(registry.loadConfiguration());
    REQUIRE(registry.loadConfiguration());

    REQUIRE(registry.getConfigurationUnsafe(COORDINATE_ID, coordinateGet));
    REQUIRE(registry.getConfigurationUnsafe(VENTING_VALVE_ATOMIC_TIMING_ID,
                                            valueInt));
    REQUIRE(registry.getConfigurationUnsafe(FLOAT_VALUE_ID, valueFloat));
    REQUIRE(registry.getConfigurationUnsafe(COORDINATE_ID, coordinateGet));

    /*! CHECK AFTER RELOAD */

    coordinateGet.latitude  = 0;
    coordinateGet.longitude = 0;
    REQUIRE(registry.getConfigurationUnsafe(COORDINATE_ID, coordinateGet));
    REQUIRE(coordinateGet.latitude == coordinatesValue.latitude);
    REQUIRE(coordinateGet.longitude == coordinatesValue.longitude);

    valueInt = 0;
    REQUIRE(registry.getConfigurationUnsafe(VENTING_VALVE_ATOMIC_TIMING_ID,
                                            valueInt));
    REQUIRE(valueInt == TEST_VALUE_UINT32);

    valueFloat = 0;
    REQUIRE(registry.getConfigurationUnsafe(FLOAT_VALUE_ID, valueFloat));
    REQUIRE(valueFloat == TEST_VALUE_FLOAT);

    /*! CANCEL CONFIGURATION */

    registry.clear();
    REQUIRE_FALSE(
        registry.getConfigurationUnsafe(COORDINATE_ID, coordinateGet));
    REQUIRE_FALSE(registry.getConfigurationUnsafe(
        VENTING_VALVE_ATOMIC_TIMING_ID, valueInt));
    REQUIRE_FALSE(registry.getConfigurationUnsafe(FLOAT_VALUE_ID, valueFloat));
}
