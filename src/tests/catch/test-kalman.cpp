/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifdef STANDALONE_CATCH1_TEST
#include "catch-tests-entry.cpp"
#endif

#include <utils/testutils/catch.hpp>
#include <iostream>
#include <src/tests/kalman/test-kalman-data.h>
#include "kalman/Kalman.h"



static MatrixBase<float, 3, 3>
        A{1, 0.2, 0.02, 0, 1, 0.2, 0, 0, 1};
static MatrixBase<float, 1, 3> C{1, 0, 0};
static MatrixBase<float, 3, 3> V1{0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
static MatrixBase<float, 1, 1> V2{10};
static MatrixBase<float, 3, 3> P0{0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1};

TEST_CASE("Update test")
{
    Kalman<3, 1> filter{A, C, V1, V2, P0};
    MatrixBase<float, 1, 1> y{};
    float T;
    float last_time = TIME[0];
    filter.X(0, 0)  = INPUT[0];
    for (unsigned i = 1; i < 101; i++)
    {
        y(0, 0)        = INPUT[i];
        T              = TIME[i] - last_time;
        filter.A(0, 1) = T;
        filter.A(0, 2) = 0.5 * T * T;
        filter.A(1, 2) = T;
        filter.update(y);
        if (filter.X(0, 0) != Approx(STATE_1[i]).epsilon(0.01))
        {
            FAIL("FAILED X(0,0) " << filter.X(0, 0) << " != " << STATE_1[i] );
        }
        else
        {
            SUCCEED();
        }
        if (filter.X(1, 0) != Approx(STATE_2[i]).epsilon(0.01))
        {
            FAIL("FAILED X(1,0) " << filter.X(1, 0) << " != " << STATE_2[i] );
        }
        else
        {
            SUCCEED();
        }
        if (filter.X(2, 0) != Approx(STATE_3[i]).epsilon(0.01))
        {
            FAIL("FAILED X(2,0) " << filter.X(2, 0) << " != " << STATE_3[i] );
        }
        else
        {
            SUCCEED();
        }


        auto predictedState = filter.state(5);
        if (predictedState(0,0) != Approx(PRED_STATE_1[i]).epsilon(0.01) ) {
            FAIL("FAILED PREDICTED X(0,0) " << predictedState(0,0) << " != " << PRED_STATE_1[i] );
        }
        else
        {
            SUCCEED();
        }
        if (predictedState(1,0) != Approx(PRED_STATE_2[i]).epsilon(0.01) ) {
            FAIL("FAILED PREDICTED X(1,0) " << predictedState(1,0) << " != " << PRED_STATE_2[i] );
        }
        else
        {
            SUCCEED();
        }
        if (predictedState(2,0) != Approx(PRED_STATE_3[i]).epsilon(0.01) ) {
            FAIL("FAILED PREDICTED X(2,0) " << predictedState(2,0) << " != " << PRED_STATE_3[i] );
        }
        else
        {
            SUCCEED();
        }

        auto predictedOutput = filter.output(5);
        if (predictedOutput(0,0) != Approx(PRED_STATE_1[i]).epsilon(0.01) ) {
            FAIL("FAILED PREDICTED Y " << predictedState(0,0) << " != " << PRED_STATE_1[i] );
        }
        else
        {
            SUCCEED();
        }
        
        
        last_time = TIME[i];
    }
}