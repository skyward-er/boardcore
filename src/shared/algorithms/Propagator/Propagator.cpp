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
#include <utils/Debug.h>

using namespace Eigen;

namespace Boardcore
{

static constexpr bool useAcceleration = true; //set true to use the propagator with acceleration

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

    if(useAcceleration) // Update Position assuming constant acceleration
    {
        // checking that last two states are not propagated
        if(state.nPropagations == 0 && oldState.nPropagations == 0)             
            state.setAcceleration((state.getVelocity() - oldState.getVelocity()) / updatePeriod);

        state.setVelocity((state.getVelocity() + state.getAcceleration()) * updatePeriod);
        state.setPosition((state.getPosition() + state.getVelocity()) * updatePeriod);

        state.nPropagations++;
        state.timestamp = TimestampTimer::getTimestamp();
    }
    else    // Update Position propagating assuming costant velocity
    { 
    state.setPosition(state.getPosition() + state.getVelocity() * updatePeriod);
    state.nPropagations++;
    state.timestamp = TimestampTimer::getTimestamp();
    }

    // Log propagator state
    PropagatorState logState(state);
    Boardcore::Logger::getInstance().log(logState);
}

void Propagator::setRocketNasState(const NASState& newRocketNasState)
{
    miosix::Lock<miosix::FastMutex> lockState(stateMutex);
    miosix::Lock<miosix::FastMutex> lock(nasStateMutex);

    // Reset nPropagations to notify another received "real" packet
    state.nPropagations = 0;
    lastRocketNasState  = newRocketNasState;
}

}  // namespace Boardcore
