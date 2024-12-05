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
#include <utils/Registry/RegistryFrontend.h>

#include <catch2/catch.hpp>
#include <cstdint>

using namespace Boardcore;

/* Magic numbers for testing the set/get for each data type, they do not
 * reflect the reals configuration ids and values in use! */
static constexpr uint32_t TEST_VALUE_UINT32              = 30;
static constexpr uint32_t TEST_VALUE_UINT32_2            = 32;
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

/**
 * @brief Test function for the forEach frontend function. Tests that the value
 * and id are correctly passed to this function
 *
 * @param entryId The id of the configuration entry
 * @param entry The EntryStructsUnion structure of the configuration entry
 */
void visitFunction(ConfigurationId entryId, EntryStructsUnion& entry)
{
    switch (entryId)
    {
        case FLOAT_VALUE_ID:
            float valf;
            entry.get(valf);
            REQUIRE(valf == TEST_VALUE_FLOAT);
            break;

        case ALGORITHM_ID:
            uint32_t valu32;
            entry.get(valu32);
            REQUIRE(valu32 == TEST_VALUE_UINT32);
            break;

        case COORDINATE_ID:
            Coordinates coordinate;
            entry.get(coordinate);
            REQUIRE(coordinate.latitude == TEST_VALUE_LATITUDE);
            REQUIRE(coordinate.longitude == TEST_VALUE_LONGITUDE);
            break;

        default:
            REQUIRE(false);
    }
};

/**
 * @brief Fake backend to test the frontend without a real backend class that
 * uses memory/storage
 */
class FakeBackend : public RegistryBackend
{
public:
    int &startCount, &saveCount, &loadCount;

    FakeBackend(int& startCount, int& saveCount, int& loadCount)
        : startCount(startCount), saveCount(saveCount), loadCount(loadCount)
    {
    }

    bool start() override
    {
        startCount++;
        return true;
    }

    bool load(std::vector<uint8_t>&) override
    {
        loadCount++;
        return true;
    }

    bool save(std::vector<uint8_t>&) override
    {
        saveCount++;
        return true;
    }
};

TEST_CASE(
    "RegistryFrontend test - Set, Get, GetOrSetDefault, forEach, isEmpty, ")
{
    RegistryFrontend registry;
    float floatValue;
    uint32_t uint32Value;
    Coordinates coordinatesValue{TEST_VALUE_LATITUDE, TEST_VALUE_LONGITUDE},
        coordinatesGet;

    // IS EMTPY

    /* Check that the registry is first empty and see that isEntryConfigured
     * works */
    REQUIRE(registry.isEmpty());
    REQUIRE_FALSE(registry.isEntryConfigured(ALGORITHM_ID));

    // GET, SET, IS CONFIGURED

    // Checks that there are effectively non-initialized entry configurations
    REQUIRE(registry.getUnsafe(static_cast<uint32_t>(DEPLOYMENT_ALTITUDE_ID),
                               floatValue) != RegistryError::OK);
    REQUIRE(registry.getUnsafe(static_cast<uint32_t>(TARGET_COORDINATES_ID),
                               coordinatesValue) != RegistryError::OK);
    REQUIRE(registry.getUnsafe(
                static_cast<uint32_t>(VENTING_VALVE_ATOMIC_TIMING_ID),
                uint32Value) != RegistryError::OK);
    REQUIRE(registry.getUnsafe(static_cast<uint32_t>(ALGORITHM_ID),
                               uint32Value) != RegistryError::OK);

    // Check set configuration results in right get
    REQUIRE(registry.setUnsafe(static_cast<uint32_t>(ALGORITHM_ID),
                               TEST_VALUE_UINT32) == RegistryError::OK);
    REQUIRE(!registry.isEmpty());

    uint32Value = 0;
    REQUIRE(registry.isEntryConfigured(ALGORITHM_ID));
    REQUIRE(registry.getUnsafe(ALGORITHM_ID, uint32Value) == RegistryError::OK);
    REQUIRE(uint32Value == TEST_VALUE_UINT32);
    uint32Value = 0;
    REQUIRE(registry.setUnsafe(COORDINATE_ID, TEST_VALUE_UINT32_2) ==
            RegistryError::OK);
    REQUIRE(registry.getUnsafe(COORDINATE_ID, uint32Value) ==
            RegistryError::OK);
    REQUIRE(uint32Value == TEST_VALUE_UINT32_2);
    uint32Value = 0;
    REQUIRE(registry.setUnsafe(COORDINATE_ID, TEST_VALUE_UINT32) ==
            RegistryError::OK);
    REQUIRE(registry.getUnsafe(COORDINATE_ID, uint32Value) ==
            RegistryError::OK);
    REQUIRE(uint32Value == TEST_VALUE_UINT32);

    /* Checks that get configuration is false if the type is incorrect w.r.t.
     * the type of the set type */
    REQUIRE_FALSE(registry.getUnsafe(COORDINATE_ID, floatValue) ==
                  RegistryError::OK);

    // GET OR SET DEFAULT TEST
    uint32Value =
        registry.getOrSetDefaultUnsafe(ALGORITHM_ID, TEST_VALUE_UINT32);
    REQUIRE(uint32Value == TEST_VALUE_UINT32);

    registry.clear();
    REQUIRE(registry.getUnsafe(ALGORITHM_ID, uint32Value) ==
            RegistryError::ENTRY_NOT_FOUND);
    uint32Value =
        registry.getOrSetDefaultUnsafe(ALGORITHM_ID, TEST_VALUE_UINT32);
    REQUIRE(uint32Value == TEST_VALUE_UINT32);
    REQUIRE(registry.getUnsafe(ALGORITHM_ID, uint32Value) == RegistryError::OK);
    REQUIRE(uint32Value == TEST_VALUE_UINT32);

    floatValue = 0;
    REQUIRE(registry.getUnsafe(FLOAT_VALUE_ID, floatValue) ==
            RegistryError::ENTRY_NOT_FOUND);
    floatValue =
        registry.getOrSetDefaultUnsafe(FLOAT_VALUE_ID, TEST_VALUE_FLOAT);
    REQUIRE(floatValue == TEST_VALUE_FLOAT);
    REQUIRE(registry.getUnsafe(FLOAT_VALUE_ID, floatValue) ==
            RegistryError::OK);
    REQUIRE(floatValue == TEST_VALUE_FLOAT);

    coordinatesGet =
        registry.getOrSetDefaultUnsafe(COORDINATE_ID, coordinatesValue);
    REQUIRE(coordinatesGet.latitude == coordinatesValue.latitude);
    REQUIRE(coordinatesGet.longitude == coordinatesValue.longitude);

    // VISIT
    registry.forEach(visitFunction);
}

TEST_CASE("RegistryFrontend test - Arm/Disarm test")
{
    RegistryFrontend registry;
    uint32_t uint32Value = 0;
    Coordinates coordinatesValue{TEST_VALUE_LATITUDE, TEST_VALUE_LONGITUDE},
        coordinateGet;
    float floatValue = 0;

    REQUIRE(registry.setUnsafe(COORDINATE_ID, coordinatesValue) ==
            RegistryError::OK);
    registry.arm();

    // If the registry is "armed" no set are allowed but gets are
    REQUIRE(registry.setUnsafe(ALGORITHM_ID, TEST_VALUE_UINT32) !=
            RegistryError::OK);
    REQUIRE(registry.getUnsafe(ALGORITHM_ID, uint32Value) != RegistryError::OK);
    REQUIRE(registry.getUnsafe(COORDINATE_ID, coordinateGet) ==
            RegistryError::OK);
    REQUIRE(coordinateGet.latitude == coordinatesValue.latitude);
    REQUIRE(coordinateGet.longitude == coordinatesValue.longitude);
    floatValue =
        registry.getOrSetDefaultUnsafe(FLOAT_VALUE_ID, TEST_VALUE_FLOAT);
    REQUIRE(floatValue == TEST_VALUE_FLOAT);
    REQUIRE(registry.getUnsafe(FLOAT_VALUE_ID, floatValue) !=
            RegistryError::OK);

    // DISARM AND SET NEW ENTRIES

    registry.disarm();
    REQUIRE(registry.setUnsafe(ALGORITHM_ID, TEST_VALUE_UINT32) ==
            RegistryError::OK);
    REQUIRE(registry.getUnsafe(ALGORITHM_ID, uint32Value) == RegistryError::OK);
    REQUIRE(uint32Value == TEST_VALUE_UINT32);
    floatValue =
        registry.getOrSetDefaultUnsafe(FLOAT_VALUE_ID, TEST_VALUE_FLOAT);
    REQUIRE(floatValue == TEST_VALUE_FLOAT);
    REQUIRE(registry.getUnsafe(FLOAT_VALUE_ID, floatValue) ==
            RegistryError::OK);
}

TEST_CASE("RegistryFrontend test - serialization/deserialization test")
{
    RegistryFrontend registry;
    Coordinates coordinatesValue{TEST_VALUE_LATITUDE, TEST_VALUE_LONGITUDE},
        coordinateGet{0, 0};
    uint32_t valueInt = 0;
    float valueFloat  = 0;

    registry.clear();
    // FIRST SET OF THE CONFIGURATION
    REQUIRE(registry.setUnsafe(COORDINATE_ID, coordinatesValue) ==
            RegistryError::OK);
    REQUIRE(registry.save() == RegistryError::OK);

    // LOAD AND CHECK CONFIGURATION
    REQUIRE(registry.save() == RegistryError::OK);
    REQUIRE(registry.getUnsafe(COORDINATE_ID, coordinateGet) ==
            RegistryError::OK);
    REQUIRE(registry.load() == RegistryError::OK);
    REQUIRE(registry.load() == RegistryError::OK);
    REQUIRE(registry.save() == RegistryError::OK);
    REQUIRE(registry.load() == RegistryError::OK);
    REQUIRE(registry.getUnsafe(COORDINATE_ID, coordinateGet) ==
            RegistryError::OK);
    REQUIRE(coordinateGet.latitude == coordinatesValue.latitude);
    REQUIRE(coordinateGet.longitude == coordinatesValue.longitude);

    // SET OTHER DATA TYPES CONFIGURATIONS ENTRIES

    REQUIRE(registry.setUnsafe((VENTING_VALVE_ATOMIC_TIMING_ID),
                               TEST_VALUE_UINT32) == RegistryError::OK);
    valueInt = 0;
    REQUIRE(registry.save() == RegistryError::OK);
    REQUIRE(registry.getUnsafe(VENTING_VALVE_ATOMIC_TIMING_ID, valueInt) ==
            RegistryError::OK);
    REQUIRE(valueInt == TEST_VALUE_UINT32);
    REQUIRE(registry.setUnsafe(FLOAT_VALUE_ID, TEST_VALUE_FLOAT) ==
            RegistryError::OK);
    valueFloat = 0;

    REQUIRE(registry.getUnsafe(VENTING_VALVE_ATOMIC_TIMING_ID, valueInt) ==
            RegistryError::OK);
    REQUIRE(valueInt == TEST_VALUE_UINT32);
    REQUIRE(registry.getUnsafe(FLOAT_VALUE_ID, valueFloat) ==
            RegistryError::OK);
    REQUIRE(valueFloat == TEST_VALUE_FLOAT);

    // SAVE AGAIN WITH ALL TYPES INSIDE CONFIGURATION

    REQUIRE(registry.getUnsafe(COORDINATE_ID, coordinateGet) ==
            RegistryError::OK);
    REQUIRE(registry.getUnsafe(COORDINATE_ID, coordinateGet) ==
            RegistryError::OK);
    REQUIRE(registry.save() == RegistryError::OK);
    REQUIRE(registry.getUnsafe(COORDINATE_ID, coordinateGet) ==
            RegistryError::OK);
    REQUIRE(registry.save() == RegistryError::OK);
    REQUIRE(registry.getUnsafe(COORDINATE_ID, coordinateGet) ==
            RegistryError::OK);
    REQUIRE(registry.load() == RegistryError::OK);
    REQUIRE(registry.getUnsafe(VENTING_VALVE_ATOMIC_TIMING_ID, valueInt) ==
            RegistryError::OK);
    REQUIRE(registry.getUnsafe(FLOAT_VALUE_ID, valueFloat) ==
            RegistryError::OK);
    REQUIRE(registry.getUnsafe(COORDINATE_ID, coordinateGet) ==
            RegistryError::OK);

    registry.save();
    registry.save();
    registry.save();
    registry.save();
    registry.save();
    registry.save();
    registry.load();

    REQUIRE(registry.getUnsafe(COORDINATE_ID, coordinateGet) ==
            RegistryError::OK);

    registry.save();
    REQUIRE(registry.load() == RegistryError::OK);
    registry.save();
    REQUIRE(registry.load() == RegistryError::OK);
    registry.save();
    REQUIRE(registry.load() == RegistryError::OK);
    REQUIRE(registry.load() == RegistryError::OK);
    REQUIRE(registry.load() == RegistryError::OK);
    REQUIRE(registry.load() == RegistryError::OK);

    REQUIRE(registry.getUnsafe(COORDINATE_ID, coordinateGet) ==
            RegistryError::OK);
    REQUIRE(registry.getUnsafe(VENTING_VALVE_ATOMIC_TIMING_ID, valueInt) ==
            RegistryError::OK);
    REQUIRE(registry.getUnsafe(FLOAT_VALUE_ID, valueFloat) ==
            RegistryError::OK);
    REQUIRE(registry.getUnsafe(COORDINATE_ID, coordinateGet) ==
            RegistryError::OK);

    // CHECK AFTER RELOAD

    coordinateGet.latitude  = 0;
    coordinateGet.longitude = 0;
    REQUIRE(registry.getUnsafe(COORDINATE_ID, coordinateGet) ==
            RegistryError::OK);
    REQUIRE(coordinateGet.latitude == coordinatesValue.latitude);
    REQUIRE(coordinateGet.longitude == coordinatesValue.longitude);

    valueInt = 0;
    REQUIRE(registry.getUnsafe(VENTING_VALVE_ATOMIC_TIMING_ID, valueInt) ==
            RegistryError::OK);
    REQUIRE(valueInt == TEST_VALUE_UINT32);

    valueFloat = 0;
    REQUIRE(registry.getUnsafe(FLOAT_VALUE_ID, valueFloat) ==
            RegistryError::OK);
    REQUIRE(valueFloat == TEST_VALUE_FLOAT);

    // CANCEL CONFIGURATION

    registry.clear();
    REQUIRE(registry.getUnsafe(COORDINATE_ID, coordinateGet) !=
            RegistryError::OK);
    REQUIRE(registry.getUnsafe(VENTING_VALVE_ATOMIC_TIMING_ID, valueInt) !=
            RegistryError::OK);
    REQUIRE(registry.getUnsafe(FLOAT_VALUE_ID, valueFloat) !=
            RegistryError::OK);
}

TEST_CASE("RegistryFrontend test - Backend test")
{
    int startCount = 0, saveCount = 0, loadCount = 0;

    RegistryFrontend registry(
        std::make_unique<FakeBackend>(startCount, saveCount, loadCount));

    REQUIRE(startCount == 0);
    REQUIRE(saveCount == 0);
    REQUIRE(loadCount == 0);

    REQUIRE(registry.start() == RegistryError::OK);
    REQUIRE(startCount == 1);
    REQUIRE(saveCount == 0);
    REQUIRE(loadCount == 0);

    REQUIRE(registry.save() == RegistryError::OK);
    REQUIRE(startCount == 1);
    REQUIRE(saveCount == 1);
    REQUIRE(loadCount == 0);

    REQUIRE(registry.load() == RegistryError::OK);
    REQUIRE(startCount == 1);
    REQUIRE(saveCount == 1);
    REQUIRE(loadCount == 1);
}
