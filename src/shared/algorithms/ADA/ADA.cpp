/* Copyright (c) 2018-2022 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli, Luca Conterio, Alberto Nidasio
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

#include "ADA.h"

#include <drivers/timer/TimestampTimer.h>
#include <utils/AeroUtils/AeroUtils.h>

namespace Boardcore
{

ADA::ADA(const ReferenceValues reference,
         const KalmanFilter::KalmanConfig kalmanConfig)
    : reference(reference), filter(kalmanConfig), state()
{
}

void ADA::update(const float pressure)
{
    const auto filterState = filter.getState();

    // Update the Kalman filter
    filter.predict();
    filter.correct(KalmanFilter::CVectorP{pressure});

    // Convert filter data to altitudes and speeds
    state.timestamp     = TimestampTimer::getTimestamp();
    state.mslAltitude   = Aeroutils::relAltitude(pressure, reference.pressure,
                                                 reference.temperature);
    state.aglAltitude   = state.mslAltitude - reference.altitude;
    state.verticalSpeed = Aeroutils::verticalSpeed(
        filterState(0), filterState(1), reference.mslPressure,
        reference.mslTemperature);
    state.x0 = filterState(0);
    state.x1 = filterState(1);
    state.x2 = filterState(2);
}

ADAState ADA::getState() { return state; }

void ADA::setReferenceValues(const ReferenceValues reference)
{
    this->reference = reference;
}

ReferenceValues ADA::getReferenceValues() { return reference; }

}  // namespace Boardcore
