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

#include <algorithms/MEA/MEA.h>

#include <catch2/catch.hpp>
#include <iostream>

#include "../algorithms/MEA/test-mea-data.h"

using namespace Boardcore;
using namespace Eigen;

constexpr float sensorNoiseVariance = 0.36f;
constexpr float modelNoiseVariance  = 0.01f;
constexpr float initialRocketMass   = 35.01f;

MEA::KalmanFilter::KalmanConfig getMEAKalmanConfig()
{
    MEA::KalmanFilter::KalmanConfig config;

    // clang-format off
    config.F = MEA::KalmanFilter::MatrixNN({
        {1.435871191228868, -0.469001276508780,  0.f}, 
        {1.f,                0.f,                0.f},
        {-0.002045309260755, 0.001867496708935,  1.f}});
    
    config.H = {1.780138883879285,-1.625379384370081,0.f};

    config.P    = MEA::KalmanFilter::MatrixNN::Zero();
    config.Q    = modelNoiseVariance * MEA::KalmanFilter::CVectorN({1, 1, 1}).asDiagonal();
    config.R[0] = sensorNoiseVariance;
    config.G    = MEA::KalmanFilter::MatrixNM{{4}, {0}, {0}};
    config.x    = {0, 0, initialRocketMass};
    // clang-format on

    return config;
}

TEST_CASE("MEA Update Test")
{
    MEA mea(getMEAKalmanConfig());
    MEAState state;

    for (unsigned i = 0; i < PRESSURE.size(); i++)
    {
        // Get the results
        state = mea.getState();

        if (state.x2 != Approx(ESTIMATED_MASS[i]).epsilon(0.01))
        {
            FAIL("The estimated mass differs from the correct one ["
                 << i << "]: " << state.x2 << " != " << ESTIMATED_MASS[i]);
        }

        if (state.correctedPressure !=
            Approx(ESTIMATED_PRESSURE[i]).epsilon(0.01))
        {
            FAIL("The estimated pressure differs from the correct one ["
                 << i << "]: " << state.correctedPressure
                 << " != " << ESTIMATED_PRESSURE[i]);
        }

        // Update the kalman
        mea.update(COMMAND[i], PRESSURE[i]);
    }
}