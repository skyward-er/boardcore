/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Pietro Bortolus
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

#include <algorithms/AirBrakes/AirBrakesInterpPID.h>

#include <algorithm>
#include <catch2/catch.hpp>
#include <fstream>
#include <iostream>

#include "../algorithms/Airbrakes/test-airbrakesInterpPID-data.h"
#include "../algorithms/Airbrakes/test-airbrakesInterpPID-references.h"

using namespace Boardcore;
using namespace std;
using namespace Boardcore::ABKInterpPID;

constexpr float MINIMUM_ALTITUDE      = 1000;
constexpr float MAXIMUM_ALTITUDE      = 3000;
constexpr float STARTING_FILTER_VALUE = 0.9f;
constexpr float ABK_CRITICAL_ALTITUDE = 2990;
constexpr float DZ                    = 10;
constexpr float INITIAL_MASS          = 29;
constexpr float DM                    = 0.4f;
constexpr float ARB_FREQ              = 10;
constexpr float PID_REF               = 0.2f;
constexpr float KP                    = 1.2f;
constexpr float KI                    = 1;
constexpr float KD                    = 0.01f;

constexpr uint16_t N_FORWARD = 0;

AirBrakesInterpPIDConfig getConfigPID()
{
    AirBrakesInterpPIDConfig config;
    config.FILTER_MINIMUM_ALTITUDE = MINIMUM_ALTITUDE;
    config.FILTER_MAXIMUM_ALTITUDE = MAXIMUM_ALTITUDE;
    config.STARTING_FILTER_VALUE   = STARTING_FILTER_VALUE;
    config.ABK_CRITICAL_ALTITUDE   = ABK_CRITICAL_ALTITUDE;
    config.DZ                      = DZ;
    config.INITIAL_MASS            = INITIAL_MASS;
    config.DM                      = DM;
    config.ARB_FREQ                = ARB_FREQ;
    config.PID_REF                 = PID_REF;
    config.KP                      = KP;
    config.KI                      = KI;
    config.KD                      = KD;
    config.N_FORWARD               = N_FORWARD;

    return config;
}

NASState getStatePID()
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
    AirBrakesInterpPID abk(
        []() { return static_cast<TimedTrajectoryPoint>(getStatePID()); },
        OPEN_TRAJECTORY_SET, CLOSED_TRAJECTORY_SET, getConfigPID(),
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

    abk.begin(30.0052959188768);

    for (size_t i = 0; i < Z.size(); i++)
        abk.update();
}
