/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <algorithms/AirBrakes/AirBrakesInterp.h>

#include <algorithm>
#include <catch2/catch.hpp>
#include <fstream>
#include <iostream>

#include "../algorithms/Airbrakes/test-airbrakesInterp-data.h"
#include "../algorithms/Airbrakes/test-airbrakesInterp-references.h"

using namespace Boardcore;
using namespace std;

constexpr float MINIMUM_ALTITUDE      = 1000;
constexpr float MAXIMUM_ALTITUDE      = 3000;
constexpr float STARTING_FILTER_VALUE = 0.9f;
constexpr float ABK_CRITICAL_ALTITUDE = 2990;
constexpr float DZ                    = 10;
constexpr float INITIAL_MASS          = 28;
constexpr float DM                    = 0.2f;
constexpr uint16_t N_FORWARD          = 2;

static const Boardcore::AirBrakesConfig ABK_CONFIG{
    0.4884,      -1.4391,    6.6940,
    -18.4272,    29.1044,    -24.5585,
    8.6058,      9.0426,     159.5995,
    4.8188,      -208.4471,  47.0771,
    1.9433e+03,  -205.6689,  -6.4634e+03,
    331.0332,    8.8763e+03, -161.8111,
    -3.9917e+03, 2.8025e-06, 0.0373,
    20,          -0.009216,  0.02492,
    -0.01627,    0.03191,    0.017671458676443,
    0,
};

AirBrakesInterpConfig getConfig()
{
    AirBrakesInterpConfig config;
    config.FILTER_MINIMUM_ALTITUDE = MINIMUM_ALTITUDE;
    config.FILTER_MAXIMUM_ALTITUDE = MAXIMUM_ALTITUDE;
    config.STARTING_FILTER_VALUE   = STARTING_FILTER_VALUE;
    config.ABK_CRITICAL_ALTITUDE   = ABK_CRITICAL_ALTITUDE;
    config.DZ                      = DZ;
    config.INITIAL_MASS            = INITIAL_MASS;
    config.DM                      = DM;
    config.N_FORWARD               = N_FORWARD;
    return config;
}

NASState getState()
{
    // Index of the progressive data point
    static size_t i = 0;

    // Max out the counter
    i = min(Z.size() - 1, i);

    NASState state;
    state.timestamp =
        i + 1;         // Increasing timestamp to let the algorithm evolve
    state.d  = -Z[i];  // Compute altitude AGL
    state.vd = -Vz[i];

    i += 1;
    return state;
}

TEST_CASE("ABK Update Test")
{
    AirBrakesInterp abk(
        []() { return static_cast<TimedTrajectoryPoint>(getState()); },
        OPEN_TRAJECTORY_SET, CLOSED_TRAJECTORY_SET, ABK_CONFIG, getConfig(),
        [&](float position)
        {
            static int i = 0;

            // Check the output
            if (position != Approx(ABK[i]).epsilon(0.01))
            {
                FAIL("The computed position differs from the correct one["
                     << i << "]: " << position << " != " << ABK[i]);
            }

            i++;
        });

    abk.begin(28.8);

    for (int i = 0; i < Z.size(); i++)
    {
        abk.update();
    }
}