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
#include <utils/Registry/RegistryFrontend.h>

#include <catch2/catch.hpp>
#include <utils/Registry/RegistryFrontend.cpp>

using namespace Boardcore;

/** Magic numbers for testing the set/get for each data type, they do not
 * reflect the reals configuration ids and values in use! */
static constexpr uint8_t testValueUint8              = 20;
static constexpr uint32_t testValueUint32            = 30;
static constexpr float testValueFloat                = 1.45;
static constexpr float testValueLatitude             = 45.50109;
static constexpr float testValueLongitude            = 9.15633;
static constexpr uint32_t coordinateId               = 8;
static constexpr uint32_t deploymentAltitudeId       = 50;
static constexpr uint32_t targetCoordinatesId        = 1;
static constexpr uint32_t ventingValveAtomicTimingId = 49;
static constexpr uint32_t algorithmId                = 128;
static constexpr uint32_t ignitionTimeId             = 3;
static constexpr uint32_t aFloatValueId              = 13;

TEST_CASE(
    "RegistryFrontend test - Set and get type Unsafe configuration entries")
{
    RegistryFrontend registry;
    float floatValue;
    uint32_t uint32Value;
    uint8_t uint8Value;
    Coordinates coordinatesValue(testValueLatitude, testValueLongitude);
    /*! Check that the registry is first empty */
    REQUIRE(registry.isConfigurationEmpty() == true);
    /*! Checks that there are effectively non-initialized entry configurations
     */
    REQUIRE(registry.getConfigurationUnsafe(
                static_cast<uint32_t>(deploymentAltitudeId), floatValue) ==
            false);
    REQUIRE(registry.getConfigurationUnsafe(
                static_cast<uint32_t>(targetCoordinatesId), coordinatesValue) ==
            false);
    REQUIRE(registry.getConfigurationUnsafe(
                static_cast<uint32_t>(ventingValveAtomicTimingId),
                uint32Value) == false);
    REQUIRE(registry.getConfigurationUnsafe(static_cast<uint32_t>(algorithmId),
                                            uint8Value) == false);
    /*! Check set configuration results in right get */
    REQUIRE(registry.setConfigurationUnsafe(static_cast<uint32_t>(algorithmId),
                                            testValueUint8) == true);
    uint8Value = 0;
    REQUIRE(registry.getConfigurationUnsafe(algorithmId, uint8Value) == true);
    REQUIRE(uint8Value == testValueUint8);
    uint32Value = 0;
    REQUIRE(registry.setConfigurationUnsafe(coordinateId, testValueUint32) ==
            true);
    REQUIRE(registry.getConfigurationUnsafe(coordinateId, uint32Value) == true);
    REQUIRE(uint32Value == testValueUint32);
    /*! Checks that get configuration is false if the type is incorrect w.r.t.
     * the type of the set type */
    REQUIRE(registry.getConfigurationUnsafe(coordinateId, floatValue) == false);
}

TEST_CASE("RegistryFrontend test - Arm/Disarm test")
{
    RegistryFrontend registry;
    uint8_t uint8Value;
    Coordinates coordinatesValue(testValueLatitude, testValueLongitude),
        coordinateGet;
    REQUIRE(registry.setConfigurationUnsafe(coordinateId, coordinatesValue) ==
            true);
    registry.arm();
    /*! If the registry is "armed" no set are allowed but gets are */
    REQUIRE(registry.setConfigurationUnsafe(algorithmId, testValueUint8) ==
            false);
    REQUIRE(registry.getConfigurationUnsafe(algorithmId, uint8Value) == false);
    REQUIRE(registry.getConfigurationUnsafe(coordinateId, coordinateGet) ==
            true);
    REQUIRE(coordinateGet.latitude == coordinatesValue.latitude);
    REQUIRE(coordinateGet.longitude == coordinatesValue.longitude);

    /*! DISARM AND SET NEW ENTRIES */

    registry.disarm();
    REQUIRE(registry.setConfigurationUnsafe(algorithmId, testValueUint8) ==
            true);
    REQUIRE(registry.getConfigurationUnsafe(algorithmId, uint8Value) == true);
    REQUIRE(uint8Value == testValueUint8);
}
TEST_CASE("RegistryFrontend test - serialization/deserialization test")
{
    RegistryFrontend registry;
    Coordinates coordinatesValue(testValueLatitude, testValueLongitude),
        coordinateGet(0, 0);
    uint32_t valueInt = 0;
    float valueFloat  = 0;

    REQUIRE(registry.setConfigurationUnsafe(coordinateId, coordinatesValue) ==
            true);
    /*! TODO: This will change to true when there is an actual backend */
    REQUIRE(registry.saveConfiguration() == false);
    {
        std::vector<uint8_t>& vector = registry.getSerializedConfiguration();
        for (int i = 0; i < 8; i++)
            REQUIRE(vector.at(i) == 0);
        REQUIRE(static_cast<int>(vector.at(8)) == 1);
        REQUIRE(static_cast<int>(vector.at(9)) == 30);
        /*! Checksum */
        // REQUIRE(static_cast<int>(vector.at(10)) == boh);
        // REQUIRE(static_cast<int>(vector.at(11)) == boh);
        // REQUIRE(static_cast<int>(vector.at(12) == boh);
        // REQUIRE(static_cast<int>(vector.at(13) == boh);
        /*! ID 8*/
        REQUIRE(static_cast<int>(vector.at(14)) == 0);
        REQUIRE(static_cast<int>(vector.at(15)) == 0);
        REQUIRE(static_cast<int>(vector.at(16)) == 0);
        REQUIRE(static_cast<int>(vector.at(17)) == 8);
        auto it = vector.begin() + 14;
        EntryStructsUnion::getFromSerializedVector(valueInt, it, vector.end());
        REQUIRE(valueInt == 8);
        /*! ID TYPE 2:coordinates*/
        REQUIRE(static_cast<int>(vector.at(18)) == 0);
        REQUIRE(static_cast<int>(vector.at(19)) == 0);
        REQUIRE(static_cast<int>(vector.at(20)) == 0);
        REQUIRE(static_cast<int>(vector.at(21)) == 2);
        /*! LATITUDE 01000010 00110110 00000001 00011110 */
        REQUIRE(static_cast<int>(vector.at(22)) == 66);
        REQUIRE(static_cast<int>(vector.at(23)) == 54);
        REQUIRE(static_cast<int>(vector.at(24)) == 1);
        REQUIRE(static_cast<int>(vector.at(25)) == 30);

        /*! LONGITUDE 01000001 00010010 10000000 01010100 */
        REQUIRE(static_cast<int>(vector.at(26)) == 65);
        REQUIRE(static_cast<int>(vector.at(27)) == 18);
        REQUIRE(static_cast<int>(vector.at(28)) == 128);
        REQUIRE(static_cast<int>(vector.at(29)) == 84);
    }

    /*! LOAD AND CHECK CONFIGURATION */

    REQUIRE(registry.loadConfiguration() == true);
    REQUIRE(registry.getConfigurationUnsafe(coordinateId, coordinateGet) ==
            true);
    REQUIRE(coordinateGet.latitude == coordinatesValue.latitude);
    REQUIRE(coordinateGet.longitude == coordinatesValue.longitude);

    /*! SET OTHER DATA TYPES CONFIGURATIONS ENTRIES*/

    REQUIRE(registry.setConfigurationUnsafe((ventingValveAtomicTimingId),
                                            testValueUint32) == true);
    valueInt = 0;
    REQUIRE(registry.getConfigurationUnsafe(ventingValveAtomicTimingId,
                                            valueInt) == true);
    REQUIRE(valueInt == testValueUint32);
    REQUIRE(registry.setConfigurationUnsafe(aFloatValueId, testValueFloat) ==
            true);
    valueFloat = 0;
    REQUIRE(registry.getConfigurationUnsafe(aFloatValueId, valueFloat) == true);
    REQUIRE(valueFloat == testValueFloat);

    /*! SAVE AGAIN WITH ALL TYPES INSIDE CONFIGURATION */

    /*! TODO: This will change to true when there is an actual backend */
    REQUIRE(registry.saveConfiguration() == false);
    REQUIRE(registry.loadConfiguration() == true);

    /*! CHECK AFTER RELOAD */

    coordinateGet.latitude  = 0;
    coordinateGet.longitude = 0;
    REQUIRE(registry.getConfigurationUnsafe(coordinateId, coordinateGet) ==
            true);
    REQUIRE(coordinateGet.latitude == coordinatesValue.latitude);
    REQUIRE(coordinateGet.longitude == coordinatesValue.longitude);

    valueInt = 0;
    REQUIRE(registry.getConfigurationUnsafe(ventingValveAtomicTimingId,
                                            valueInt) == true);
    REQUIRE(valueInt == testValueUint32);

    valueFloat = 0;
    REQUIRE(registry.getConfigurationUnsafe(aFloatValueId, valueFloat) == true);
    REQUIRE(valueFloat == testValueFloat);

    /*! CANCEL CONFIGURATION */

    registry.clear();
    REQUIRE(registry.getConfigurationUnsafe(coordinateId, coordinateGet) ==
            false);
    REQUIRE(registry.getConfigurationUnsafe(ventingValveAtomicTimingId,
                                            valueInt) == false);
    REQUIRE(registry.getConfigurationUnsafe(aFloatValueId, valueFloat) ==
            false);
}
