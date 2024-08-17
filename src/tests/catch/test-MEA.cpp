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

/*
constexpr float sensorNoiseVariance = 0.36f;
constexpr float modelNoiseVariance  = 0.1f;
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
    config.Q    = modelNoiseVariance * MEA::KalmanFilter::CVectorN({1, 1,
1}).asDiagonal(); config.R[0] = sensorNoiseVariance; config.G    =
MEA::KalmanFilter::MatrixNM{{4}, {0}, {0}}; config.x    = {0, 0,
initialRocketMass};
    // clang-format on

    return config;
}
*/

MEA::Config getMEAConfig()
{
    MEA::Config config;

    // clang-format off
    config.F = Matrix<float, 3, 3>({
        {1.435871191228868, -0.469001276508780,  0.f}, 
        {1.f,                0.f,                0.f},
        {-0.002045309260755, 0.001867496708935,  1.f}});
    config.Q = Matrix<float, 3, 3>({
        {0.1f, 0.0f, 0.0f}, 
        {0.0f, 0.1f, 0.0f},
        {0.0f, 0.0f, 0.1f}});
    config.G = Matrix<float, 3, 1>{{4}, {0}, {0}};
    
    config.baroH = {1.780138883879285,-1.625379384370081,0.f};
    config.baroR = 0.36f;

    config.P           = Matrix<float, 3, 3>::Zero();
    config.initialMass = 35.01f;
    // clang-format on

    return config;
}

TEST_CASE("MEA Update Test")
{
    MEA mea(getMEAConfig());
    MEAState state;

    std::cout << MEAState::header();

    for (unsigned i = 1; i < PRESSURE.size(); i++)
    {
        // Update the kalman
        MEA::Step step{COMMAND[i - 1]};
        step.withCCPressure(PRESSURE[i]);

        mea.update(step);

        // Get the results
        state = mea.getState();

        state.print(std::cout);

        if (state.estimatedMass != Approx(ESTIMATED_MASS[i]).epsilon(0.01))
        {
            FAIL("The estimated mass differs from the correct one ["
                 << i << "]: " << state.estimatedMass
                 << " != " << ESTIMATED_MASS[i]);
        }

        if (state.estimatedPressure !=
            Approx(ESTIMATED_PRESSURE[i]).epsilon(0.01))
        {
            FAIL("The estimated pressure differs from the correct one ["
                 << i << "]: " << state.estimatedPressure
                 << " != " << ESTIMATED_PRESSURE[i]);
        }
    }
}