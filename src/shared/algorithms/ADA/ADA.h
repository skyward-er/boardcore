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

#pragma once

#include <algorithms/Kalman/Kalman.h>
#include <algorithms/ReferenceValues.h>

#include "ADAData.h"

namespace Boardcore
{

class ADA
{
public:
    using KalmanFilter = Kalman<float, 3, 1>;

    explicit ADA(const KalmanFilter::KalmanConfig kalmanConfig);

    /**
     * @brief Update the Kalman filter.
     *
     * @param pressure Measured pressure [Pa].
     */
    void update(const float pressure);

    /**
     * @param state ADA state with altitude and Kalman parameters.
     */
    ADAState getState();

    /**
     * @brief Changes the reference values.
     */
    void setReferenceValues(const ReferenceValues reference);

    /**
     * @brief Returns the current reference values.
     */
    ReferenceValues getReferenceValues();

private:
    ReferenceValues reference;

    KalmanFilter filter;

    ADAState state;
};

}  // namespace Boardcore
