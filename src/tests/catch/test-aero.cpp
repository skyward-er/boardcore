/* Copyright (c) 2019-2023 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Emilio Corigliano
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

TEST_CASE("[AeroUtils] computeRho")
{
    {
        // Test for same t0 of atmosisa
        const float t0 = 288.15;
        float d[]      = {0, -500, -1000, -1500, -2000, -2500, -3000};

        float rho[] = {1.2250, 1.1673, 1.1116, 1.0581, 1.0065, 0.9569, 0.9091};

        for (int i = 0; i < sizeof(d) / sizeof(float); i++)
            REQUIRE(computeRho(d[i], t0) == Approx(rho[i]).epsilon(0.001));
    }

    {
        // Test for a different t0 from atmosisa
        const float t0 = 287.11;
        float d[]      = {0,         -315.556,  -631.111,  -946.667,  -1262.222,
                          -1577.778, -1893.333, -2208.889, -2524.444, -2840};

        float rho[] = {1.2062937204963,  1.17004184985014,  1.13462949957852,
                       1.10004288170165, 1.06626877484746,  1.03329364698175,
                       1.0011045116218,  0.969688091422233, 0.939031634200062,
                       0.909122116038058};

        for (int i = 0; i < sizeof(d) / sizeof(float); i++)
            REQUIRE(computeRho(d[i], t0) == Approx(rho[i]).epsilon(0.001));
    }
}

TEST_CASE("[AeroUtils] computeSoundSpeed")
{
    {
        // Test for same t0 of atmosisa
        const float t0 = 288.15;
        float d[]      = {0, -500, -1000, -1500, -2000, -2500, -3000};

        float c[] = {340.2941, 338.3696, 336.4341, 334.4874,
                     332.5293, 330.5596, 328.5781};

        for (int i = 0; i < sizeof(d) / sizeof(float); i++)
            REQUIRE(computeSoundSpeed(d[i], t0) == Approx(c[i]).epsilon(0.001));
    }

    {
        // Test for a different t0 from atmosisa
        const float t0 = 287.11;
        float d[]      = {0,         -315.556,  -631.111,  -946.667,  -1262.222,
                          -1577.778, -1893.333, -2208.889, -2524.444, -2840};

        float c[] = {339.679469143191, 338.463959192682, 337.244072148872,
                     336.019752566034, 334.790959617651, 333.557636034153,
                     332.319739232958, 331.077210026372, 329.830003980861,
                     328.578059889884};

        for (int i = 0; i < sizeof(d) / sizeof(float); i++)
            REQUIRE(computeSoundSpeed(d[i], t0) == Approx(c[i]).epsilon(0.001));
    }
}

TEST_CASE("[AeroUtils] computeMach")
{
    {
        // Test for same t0 of atmosisa
        const float t0 = 288.15;
        float d[]      = {0, -500, -1000, -1500, -2000, -2500, -3000, -3000};
        float vtot[]   = {0, 100, 100, 100, 100, 100, 100, 0};

        float mach[] = {0, 0.2955, 0.2972, 0.2990, 0.3007, 0.3025, 0.3043, 0};

        for (int i = 0; i < sizeof(d) / sizeof(float); i++)
            REQUIRE(computeMach(d[i], vtot[i], t0) ==
                    Approx(mach[i]).epsilon(0.001));
    }

    {

        // Test for a different t0 from atmosisa
        const float t0 = 287.11;
        float d[]      = {0,         -315.556,  -631.111,  -946.667,  -1262.222,
                          -1577.778, -1893.333, -2208.889, -2524.444, -2840};
        float vtot[]   = {0,
                          156.25199959656,
                          312.503999193119,
                          468.755998789678,
                          625.007998386238,
                          1250.01599677248,
                          1031.26319733729,
                          812.510397902109,
                          593.757598466926,
                          375.004799031743};

        float mach[] = {0,
                        0.461650333374514,
                        0.926640451237251,
                        1.39502512935622,
                        1.86686044061653,
                        3.74752624953993,
                        3.10322582618053,
                        2.45414173279215,
                        1.80019280023227,
                        1.14129591962841};

        for (int i = 0; i < sizeof(d) / sizeof(float); i++)
            REQUIRE(computeMach(d[i], vtot[i], t0) ==
                    Approx(mach[i]).epsilon(0.001));
    }
}