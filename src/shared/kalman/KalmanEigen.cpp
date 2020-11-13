/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Alessandro Del Duca, Luca Conterio
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

#include "KalmanEigen.h"

KalmanEigen::KalmanEigen(const KalmanConfig& config)
    : n(config.F.rows()), m(config.G.cols()), p(config.H.rows()), F(config.F),
      G(config.G), H(config.H), Q(config.Q), R(config.R), P(config.P), S(n, n),
      K(p, n), I(n, n), x(n), y_hat(p)  // , u(m)
{
    x.setZero();
    // u.setZero();
    I.setIdentity();
}

void KalmanEigen::init(const VectorXf& x0)  // const VectorXf& u0
{
    // static_assert(A.rows() == A.cols(), "Matrix A must be a square matrix.");
    // static_assert(B.rows() == A.rows(), " ");
    // static_assert(A.cols() == C.cols(),
    //              "Matrix C must have the same number of columns as A.");

    x = x0;
    // u = u0; // useless if u is passed at each step to predict()
}

void KalmanEigen::predict()
{
    P = F * P * F.transpose() + Q;
    x = F * x;
}

void KalmanEigen::predict(const VectorXf& u)
{
    P = F * P * F.transpose() + Q;
    x = F * x + G * u;
}

bool KalmanEigen::correct(const VectorXf& y)
{
    S = H * P * H.transpose() + R;

    // here the determinant is computed and
    // then the inverse recomputes it,
    // this passage could be optimized
    if (S.determinant() < 1e-3)
    {
        return false;
    }

    K = P * H.transpose() * S.inverse();
    P = (I - K * H) * P;

    x = x + K * (y - H * x);

    return true;
}

const VectorXf& KalmanEigen::getState() { return x; }

const VectorXf& KalmanEigen::getOutput()
{
    y_hat = H * x;
    return y_hat;
}

const VectorXf KalmanEigen::predictOutput(uint32_t k)
{
    return H * predictState(k);
}

// only without u
const VectorXf KalmanEigen::predictState(uint32_t k)
{
    VectorXf x_hat(n);
    x_hat = x;
    for (uint32_t i = 0; i < k; i++)
    {
        x_hat = F * x_hat;
    }
    return x_hat;
}