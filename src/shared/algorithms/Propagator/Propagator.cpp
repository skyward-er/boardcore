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

#include "Propagator.h"

#include <drivers/timer/TimestampTimer.h>
#include <utils/AeroUtils/AeroUtils.h>

using namespace Eigen;

namespace Boardcore
{

Propagator::Propagator(float updatePeriod) : updatePeriod(updatePeriod), state()
{
}

bool Propagator::init() { return true; }

void Propagator::step()
{
    // Take new rocket data only if it has been just updated, otherwise take
    // last state available
    const PropagatorState& oldState = getState();

    PropagatorState newState =
        (oldState.nPropagations == 0
             ? PropagatorState(oldState.timestamp, oldState.nPropagations,
                               getRocketNasState())
             : oldState);

    // Update Position propagating it with velocity
    newState.x_prop = newState.x_prop + newState.v_prop * updatePeriod;
    newState.nPropagations++;
    newState.timestamp = TimestampTimer::getTimestamp();

    // set propagated state
    setState(newState);
}

void Propagator::setRocketNasState(const NASState& newRocketNasState)
{
    miosix::Lock<miosix::FastMutex> lock(nasStateMutex);

    // Reset nPropagations to notify another received "real" packet
    state.nPropagations = 0;
    lastRocketNasState  = newRocketNasState;
}

}  // namespace Boardcore
