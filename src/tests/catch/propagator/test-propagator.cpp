/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <algorithms/Propagator/Propagator.h>

#include <catch2/catch.hpp>
#include <iostream>

#include "test-propagator-data.h"

using namespace Boardcore;
using namespace Eigen;

void checkPropagatorStates(const PropagatorState& currentState,
                           const PropagatorState& expectedState)
{
    REQUIRE(currentState.nPropagations == expectedState.nPropagations);
    REQUIRE(currentState.getNasState().n ==
            Approx(expectedState.getNasState().n).epsilon(0.0001));
    REQUIRE(currentState.getNasState().e ==
            Approx(expectedState.getNasState().e).epsilon(0.0001));
    REQUIRE(currentState.getNasState().d ==
            Approx(expectedState.getNasState().d).epsilon(0.0001));
    REQUIRE(currentState.getNasState().vn ==
            Approx(expectedState.getNasState().vn).epsilon(0.0001));
    REQUIRE(currentState.getNasState().ve ==
            Approx(expectedState.getNasState().ve).epsilon(0.0001));
    REQUIRE(currentState.getNasState().vd ==
            Approx(expectedState.getNasState().vd).epsilon(0.0001));
    REQUIRE(currentState.getNasState().qx ==
            Approx(expectedState.getNasState().qx).epsilon(0.0001));
    REQUIRE(currentState.getNasState().qy ==
            Approx(expectedState.getNasState().qy).epsilon(0.0001));
    REQUIRE(currentState.getNasState().qz ==
            Approx(expectedState.getNasState().qz).epsilon(0.0001));
    REQUIRE(currentState.getNasState().qw ==
            Approx(expectedState.getNasState().qw).epsilon(0.0001));
    REQUIRE(currentState.getNasState().bx ==
            Approx(expectedState.getNasState().bx).epsilon(0.0001));
    REQUIRE(currentState.getNasState().by ==
            Approx(expectedState.getNasState().by).epsilon(0.0001));
    REQUIRE(currentState.getNasState().bz ==
            Approx(expectedState.getNasState().bz).epsilon(0.0001));
}

void testPropagator(float dt, const PropagatorState& STATE0, NASState* nas,
                    uint32_t n)
{
    // Setting up the Propagator
    Propagator propagator(dt);
    propagator.init();
    propagator.begin();

    for (uint32_t i = 0; i < n; i++)
    {
        // Calculating the expected state after the operation
        PropagatorState expectedState(i, i % 10, nas[i]);

        if (i % 10 == 0)
        {
            // Simulating a reception of a packet with the nas state; it doesn't
            // update the internal state
            propagator.setRocketNasState(expectedState.getNasState());
        }
        else
        {
            // Propagating the internal state of the propagator
            propagator.update();
            checkPropagatorStates(propagator.getState(), expectedState);
        }
    }
}

TEST_CASE("Propagator Update Test 0")
{
    using namespace TestPropagator0;

    testPropagator(dt, STATE0, nas, n);
}

TEST_CASE("Propagator Update Test 1")
{
    using namespace TestPropagator1;

    testPropagator(dt, STATE0, nas, n);
}

TEST_CASE("Propagator Update Test 2")
{
    using namespace TestPropagator2;

    testPropagator(dt, STATE0, nas, n);
}

TEST_CASE("Propagator Update Test 3")
{
    using namespace TestPropagator3;

    testPropagator(dt, STATE0, nas, n);
}