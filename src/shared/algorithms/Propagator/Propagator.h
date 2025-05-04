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

#pragma once

#include <algorithms/Algorithm.h>
#include <logger/Logger.h>
#include <miosix.h>

#include <chrono>

#include "PropagatorData.h"
#include "algorithms/NAS/NASState.h"
#include "sensors/SensorData.h"

namespace Boardcore
{

/**
 * @brief Predictor class that linearly propagates the last available rocket
 * position by means of the rocket NAS velocity.
 */
class Propagator : public Algorithm
{
public:
    /**
     * @brief Constructor of the propagator class.
     *
     * @param updatePeriod The period of update of the predictor algorithm [ms].
     */
    explicit Propagator(std::chrono::milliseconds updatePeriod);

    /**
     * @brief Dummy init since we don't have to setup anything.
     * @return True.
     */
    bool init() override;

    /**
     * @brief Synchronized setter for the latest rocket nas state. Also notifies
     * the predictor of a new packet arrival.
     *
     * @param newRocketNasState The updated NAS state of the rocket.
     */
    void setRocketNasState(const NASState& newRocketNasState);

    /**
     * @brief Synchronized getter for the last rocket NAS State passed to the
     * propagator.
     *
     * @return The last NAS state of the rocket.
     */
    inline NASState getRocketNasState()
    {
        miosix::Lock<miosix::FastMutex> lock(nasStateMutex);
        return lastRocketNasState;
    }

    /**
     * @brief Synchronized getter for the State of the predictor.
     *
     * @return The state of the predictor.
     */
    inline PropagatorState getState()
    {
        miosix::Lock<miosix::FastMutex> lock(stateMutex);
        return state;
    }

private:
    /**
     * @brief Uses the previously set data to update the state of the predictor.
     */
    void step() override;

    float updatePeriod;               ///< Update period of the propagator [s]
    PropagatorState state;            ///< State of the predictor
    NASState lastRocketNasState;      ///< Last received rocket NAS state
    miosix::FastMutex nasStateMutex;  ///< mutex to sync nasState accesses
    miosix::FastMutex stateMutex;     ///< mutex to sync state accesses
    Eigen::Vector3f last_real_velocity;  ///< last non-propagated velocity
    uint64_t t0 = 0, t1 = 0,
             dt = 0;  ///< time values used to compute acceleration [u]
    const uint64_t maxAccelerationTime =
        5000000;  ///< Time limit after which acceleration data is ignored for
                  ///< propagation (5s)
};

}  // namespace Boardcore
