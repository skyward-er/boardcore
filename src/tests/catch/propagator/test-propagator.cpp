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

#include <algorithms/Propagator/propagator.h>

#include <catch2/catch.hpp>

#include "test-propagator-data.h"

using namespace Boardcore;
using namespace Eigen;

TEST_CASE("Propagator Update Test 1")
{
    using namespace TestPropagator1;
    PropagatorState state;

    for (int i = 1; i < 50; i++)
    {
        float updatePeriod = i * 10.f;
        Propagator propagator(updatePeriod);
        propagator.init();
        propagator.setState(STATE0);

        // Check if setState works
        if (propagator.getState() != STATE0)
        {
            FAIL(
                "The stored rocket nas state is different than the set one "
                "["
                << i << "]: " << propagator.getState() << " != " << STATE0);
        }

        // Update of the state
        propagator.update();
        {
            PropagatorState STATE1(STATE0);
            STATE1.nPropagations++;
            if (propagator.getState() != STATE1)
            {
                FAIL(
                    "The updated nas state is different than the correct one "
                    "["
                    << i << "]: " << propagator.getState() << " != " << STATE1);
            }
        }

        for (unsigned i = 1; i < ROCKET_STATES.size(); i++)
        {
            propagator.setRocketNasState(ROCKET_STATES[i]);
            // check if the nas state of the rocket is stored properly
            if (propagator.getRocketNasState() != ROCKET_STATES[i])
            {
                FAIL(
                    "The stored rocket nas state is different than the set one "
                    "["
                    << i << "]: " << state << " != " << ROCKET_STATES[i]);
            }

            // check if the nPropagations value is reset
            if (propagator.getState().nPropagations != 0)
            {
                FAIL(
                    "The stored rocket nas state is different than the set one "
                    "["
                    << i << "]: " << state << " != " << ROCKET_STATES[i]);
            }

            PropagatorState newState = ROCKET_STATES[i];

            propagator.update();
            newState.nPropagations++;
            newState.x_prop += (newState.v_prop * updatePeriod);
            if (propagator.getState() !=
                Approx(newState).epsilon(0.01))
            {
                FAIL(
                    "The estimated rocket state is different than the correct "
                    "one ["
                    << i << "]: " << propagator.getState()
                    << " != " << newState);
            }

            propagator.update();
            newState.nPropagations++;
            newState.x_prop += (newState.v_prop * updatePeriod);
            if (propagator.getState() !=
                Approx(newState).epsilon(0.01))
            {
                FAIL(
                    "The estimated rocket state is different than the correct "
                    "one ["
                    << i << "]: " << propagator.getState()
                    << " != " << newState);
            }
        }
    }
}