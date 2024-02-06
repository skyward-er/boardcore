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

static constexpr uint8_t testValueUint8   = 20;
static constexpr uint32_t testValueUint32 = 30;
static constexpr float testValueFloat     = 1.45;

TEST_CASE("RegistryFrontend test - Set and get configuration entries")
{
    RegistryFrontend registry;
    float floatValue;
    uint32_t uint32Value;
    uint8_t uint8Value;
    Coordinates coordinatesValue;
    /*! Check that the registry is first empty */
    REQUIRE(registry.isConfigurationEmpty() == true);
    /*! Checks that there are effectively non-initialized entry configurations
     */
    REQUIRE(registry.getConfigurationUnsafe(
                static_cast<uint32_t>(ConfigurationEnum::DEPLOYMENT_ALTITUDE),
                &floatValue) == false);
    REQUIRE(registry.getConfigurationUnsafe(
                static_cast<uint32_t>(ConfigurationEnum::TARGET_COORDINATES),
                &coordinatesValue) == false);
    REQUIRE(registry.getConfigurationUnsafe(
                static_cast<uint32_t>(
                    ConfigurationEnum::VENTING_VALVE_ATOMIC_TIMING),
                &uint32Value) == false);
    REQUIRE(registry.getConfigurationUnsafe(
                static_cast<uint32_t>(ConfigurationEnum::ALGORITHM),
                &uint8Value) == false);
    /*! Check set configuration results in right get */
    REQUIRE(registry.setConfigurationUnsafe(
                static_cast<uint32_t>(ConfigurationEnum::ALGORITHM),
                testValueUint8) == true);
    uint8Value = 0;
    REQUIRE(registry.getConfigurationUnsafe(ConfigurationEnum::ALGORITHM,
                                            &uint8Value) == true);
    REQUIRE(uint8Value == testValueUint8);
    uint32Value = 0;
    REQUIRE(registry.setConfigurationUnsafe(100, testValueUint32) == true);
    REQUIRE(registry.getConfigurationUnsafe(100, &uint32Value) == true);
    REQUIRE(uint32Value == testValueUint32);
}
