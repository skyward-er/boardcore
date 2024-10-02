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

Propagator::Propagator(std::chrono::milliseconds pUpdatePeriod)
    : updatePeriod(static_cast<float>(pUpdatePeriod.count()) / 1000), state()
{
}

bool Propagator::init() { return true; }

void Propagator::step()
{
    miosix::Lock<miosix::FastMutex> lock(stateMutex);
    // Take new rocket data only if it has been just updated, otherwise take
    // last state available
    PropagatorState oldState = state;

    // updates with the last received NAS state if present, otherwise uses the
    // last Propagator state
    state = (oldState.nPropagations == 0
                 ? PropagatorState(oldState.timestamp, oldState.nPropagations,
                                   getRocketNasState())
                 : oldState);

    // Update Position propagating it with velocity
    state.x_prop = state.x_prop + state.v_prop * updatePeriod;
    state.nPropagations++;
    state.timestamp = TimestampTimer::getTimestamp();
}

void Propagator::setRocketNasState(const NASState& newRocketNasState)
{
    miosix::Lock<miosix::FastMutex> lockState(stateMutex);
    miosix::Lock<miosix::FastMutex> lockNAS(nasStateMutex);

    // Reset nPropagations to notify another received "real" packet
    state.nPropagations = 0;
    lastRocketNasState  = newRocketNasState;
}

}  // namespace Boardcore
