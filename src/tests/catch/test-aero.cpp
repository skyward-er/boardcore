/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <utils/AeroUtils/AeroUtils.h>

#include <catch2/catch.hpp>

using namespace Boardcore;
using namespace Aeroutils;

float mslAltitude(float pressure, float pressureRef, float temperatureRef,
                  float zRef)
{
    using namespace Aeroutils;
    float t0 = mslTemperature(temperatureRef, zRef);

    return relAltitude(pressure, mslPressure(pressureRef, temperatureRef, zRef),
                       t0);
}

TEST_CASE("[AeroUtils] relAltitude")
{
    REQUIRE(mslAltitude(101325, 101325, 288.150, 0) ==
            Approx(0).epsilon(0.0001));
    REQUIRE(mslAltitude(100726, 100726, 287.825, 50) ==
            Approx(50).epsilon(0.0001));
    REQUIRE(mslAltitude(89874.6, 89874.6, 281.650, 1000) ==
            Approx(1000).epsilon(0.0001));
    REQUIRE(mslAltitude(22632.1, 22632.1, 216.650, 11000) ==
            Approx(11000).epsilon(0.0001));
    REQUIRE(mslAltitude(102532, 102532, 288.800, -100) ==
            Approx(-100).epsilon(0.0001));
}

TEST_CASE("[AeroUtils] relPressure")
{
    REQUIRE(relPressure(35) == Approx(100905).epsilon(0.0001));
    REQUIRE(relPressure(143) == Approx(99618).epsilon(0.0001));
    REQUIRE(relPressure(450) == Approx(96034).epsilon(0.0001));
    REQUIRE(relPressure(1765) == Approx(81842).epsilon(0.0001));
    REQUIRE(relPressure(2210) == Approx(77444).epsilon(0.0001));
}

TEST_CASE("[AeroUtils] relDensity")
{

    REQUIRE(relDensity(101325) == Approx(1.225).epsilon(0.0001));
    REQUIRE(relDensity(100129.438691069) ==
            Approx(1.21328277727309).epsilon(0.0001));
    REQUIRE(relDensity(89874.5715517214) ==
            Approx(1.11164259066989).epsilon(0.0001));
    REQUIRE(relDensity(70108.5471843675) ==
            Approx(0.909122116038058).epsilon(0.0001));
    REQUIRE_FALSE(relDensity(101325) == Approx(1.226).epsilon(0.0001));
}

TEST_CASE("[AeroUtils] mslPressure")
{
    // 101325 Pa, 0.01% error allowed
    Approx isa_P0 = Approx(101325).epsilon(0.0001);

    // Test against various ISA altitudes
    REQUIRE(mslPressure(101325, 288.15, 0) == isa_P0);
    REQUIRE(mslPressure(100725.8, 287.825, 50) == isa_P0);
    REQUIRE(mslPressure(89874.6, 281.65, 1000) == isa_P0);
    REQUIRE(mslPressure(22632.1, 216.65, 11000) == isa_P0);
    REQUIRE(mslPressure(102531.8, 288.8, -100) == isa_P0);
}

TEST_CASE("[AeroUtils] mslTemperature")
{
    // 15 deg celsius, 0.01% error allowed
    Approx isa_T0 = Approx(288.151).margin(0.001);

    // Test against various ISA altitudes
    REQUIRE(mslTemperature(288.15, 0) == isa_T0);
    REQUIRE(mslTemperature(287.825, 50) == isa_T0);
    REQUIRE(mslTemperature(281.65, 1000) == isa_T0);
    REQUIRE(mslTemperature(216.65, 11000) == isa_T0);
    REQUIRE(mslTemperature(288.8, -100) == isa_T0);
}

TEST_CASE("[AeroUtils] verticalSpeed")
{
    const int count = 5;
    float p[]       = {100129.4, 99555.8, 89153.1, 23611.1, 101284.6};

    float dpDt[] = {-114.98, -114.45, -104.66, -35.691, -116.05};

    Approx targetVSpeed = Approx(10).epsilon(0.001);

    for (int i = 0; i < count; i++)
        REQUIRE(verticalSpeed(p[i], dpDt[i], 100129.4, 297.5) == targetVSpeed);
}
