/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Author: Luca Mozzarelli
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

#ifdef STANDALONE_CATCH1_TEST
#include "catch-tests-entry.cpp"
#endif

#define EIGEN_RUNTIME_NO_MALLOC

#include <src/tests/kalman/test-kalman-data.h>

#include <iostream>
#include <utils/testutils/catch.hpp>

#include "kalman/KalmanEigen.h"

using namespace Eigen;

static const uint8_t STATES_DIM  = 3;
static const uint8_t OUTPUTS_DIM = 1;

static const Matrix<float, STATES_DIM, STATES_DIM> F =
    (Matrix<float, STATES_DIM, STATES_DIM>(STATES_DIM, STATES_DIM) << 1, 0.2,
     0.02, 0, 1, 0.2, 0, 0, 1)
        .finished();
// Output matrix
static const Matrix<float, OUTPUTS_DIM, STATES_DIM> H{1, 0, 0};

// Initial error covariance matrix
static const Matrix<float, STATES_DIM, STATES_DIM> P =
    (Matrix<float, STATES_DIM, STATES_DIM>(STATES_DIM, STATES_DIM) << 0.1, 0, 0,
     0, 0.1, 0, 0, 0, 0.1)
        .finished();
// Model variance matrix
static const Matrix<float, STATES_DIM, STATES_DIM> Q =
    (Matrix<float, STATES_DIM, STATES_DIM>(STATES_DIM, STATES_DIM) << 0.01, 0,
     0, 0, 0.01, 0, 0, 0, 0.01)
        .finished();
// Measurement variance
static const Matrix<float, OUTPUTS_DIM, OUTPUTS_DIM> R{10};
// State vector
static const Matrix<float, STATES_DIM, 1> x0(INPUT[0], 0.0, 0.0);

static const KalmanEigen<float, STATES_DIM, OUTPUTS_DIM>::KalmanConfig
getKalmanConfig()
{
    KalmanEigen<float, STATES_DIM, OUTPUTS_DIM>::KalmanConfig config;
    config.F = F;
    config.H = H;
    config.Q = Q;
    config.R = R;
    config.P = P;
    config.x = x0;

    return config;
}

TEST_CASE("Update test")
{
    KalmanEigen<float, STATES_DIM, OUTPUTS_DIM> filter(getKalmanConfig());

    Matrix<float, OUTPUTS_DIM, 1> y{};
    float T;
    float last_time = TIME[0];

    for (unsigned i = 1; i < 101; i++)
    {
        // printf("i = %d \n", i);

        y(0, 0) = INPUT[i];
        T       = TIME[i] - last_time;

        Matrix<float, STATES_DIM, STATES_DIM> F_new;
        F_new << F;
        F_new(0, 1) = T;
        F_new(0, 2) = 0.5 * T * T;
        F_new(1, 2) = T;

        filter.predict(F_new);

        if (!filter.correct(y))
        {
            FAIL("Correction failed at iteration : %d \n", i);
        }

        if (filter.getState()(0, 0) != Approx(STATE_1[i]).epsilon(0.01))
        {
            FAIL("FAILED X(0,0) " << filter.getState()(0, 0)
                                  << " != " << STATE_1[i]);
        }
        else
        {
            SUCCEED();
        }
        if (filter.getState()(1, 0) != Approx(STATE_2[i]).epsilon(0.01))
        {
            FAIL("FAILED X(1,0) " << filter.getState()(1, 0)
                                  << " != " << STATE_2[i]);
        }
        else
        {
            SUCCEED();
        }
        if (filter.getState()(2, 0) != Approx(STATE_3[i]).epsilon(0.01))
        {
            FAIL("FAILED X(2,0) " << filter.getState()(2, 0)
                                  << " != " << STATE_3[i]);
        }
        else
        {
            SUCCEED();
        }

        auto predictedState = filter.predictState(5);
        if (predictedState(0, 0) != Approx(PRED_STATE_1[i]).epsilon(0.01))
        {
            FAIL("FAILED PREDICTED X(0,0) " << predictedState(0, 0)
                                            << " != " << PRED_STATE_1[i]);
        }
        else
        {
            SUCCEED();
        }
        if (predictedState(1, 0) != Approx(PRED_STATE_2[i]).epsilon(0.01))
        {
            FAIL("FAILED PREDICTED X(1,0) " << predictedState(1, 0)
                                            << " != " << PRED_STATE_2[i]);
        }
        else
        {
            SUCCEED();
        }
        if (predictedState(2, 0) != Approx(PRED_STATE_3[i]).epsilon(0.01))
        {
            FAIL("FAILED PREDICTED X(2,0) " << predictedState(2, 0)
                                            << " != " << PRED_STATE_3[i]);
        }
        else
        {
            SUCCEED();
        }

        auto predictedOutput = filter.predictOutput(5);
        if (predictedOutput(0, 0) != Approx(PRED_STATE_1[i]).epsilon(0.01))
        {
            FAIL("FAILED PREDICTED Y " << predictedState(0, 0)
                                       << " != " << PRED_STATE_1[i]);
        }
        else
        {
            SUCCEED();
        }

        last_time = TIME[i];
    }
}