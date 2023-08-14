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

#pragma once

#include <algorithms/Kalman/Kalman.h>

#include "MEAData.h"

namespace Boardcore
{

/**
 * @brief MEA stands for Mass Estimation Algorithm.
 *
 * It represents a kalman filter which by performing a prediction with the
 * current feed valve state and a correction with the pressure in combustion
 * chamber, estimates the remaining mass of the rocket.
 */
class MEA
{
public:
    using KalmanFilter = Kalman<float, 3, 1, 1>;

    explicit MEA(const KalmanFilter::KalmanConfig kalmanConfig);

    /**
     * @brief Update the Kalman filter.
     *
     * @param feedValvePosition Position reference of the feed valve [0-1]
     * @param pressure Measured pressure inside the combustion chamber [Pa]
     */
    void update(const float feedValvePosition, const float pressure);

    /**
     * @brief Returns the MEA data including the estimated mass.
     */
    MEAState getState();

    /**
     * @brief Sets the Kalman filter configuration.
     */
    void setKalmanConfig(KalmanFilter::KalmanConfig config);

private:
    /**
     * @brief Computes useful data from the kalman current state.
     */
    void updateState();

    KalmanFilter filter;
    MEAState state;
};

}  // namespace Boardcore
