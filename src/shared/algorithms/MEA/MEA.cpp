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

#include "MEA.h"

#include <drivers/timer/TimestampTimer.h>

namespace Boardcore
{
MEA::MEA(const KalmanFilter::KalmanConfig kalmanConfig)
    : filter(kalmanConfig), state()
{
    updateState();
}

void MEA::update(const float feedValvePosition, const float pressure)
{
    // Update the Kalman filter
    filter.predictWithControl(KalmanFilter::CVectorM{feedValvePosition});
    filter.correct(KalmanFilter::CVectorP{pressure});

    // Compute the derived values for useful data
    updateState();
}

MEAState MEA::getState() { return state; }

void MEA::setKalmanConfig(KalmanFilter::KalmanConfig config)
{
    filter.setConfig(config);
}

void MEA::updateState()
{
    const auto filterState  = filter.getState();
    const auto filterOutput = filter.getOutput();

    state.timestamp         = TimestampTimer::getTimestamp();
    state.correctedPressure = filterOutput(0);
    state.x0                = filterState(0);
    state.x1                = filterState(1);
    state.x2                = filterState(2);
}
}  // namespace Boardcore