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

#include "ExtendedKalmanEigen.h"

ExtendedKalmanEigen::ExtendedKalmanEigen(const ExtendedKalmanConfig& config)
    : n(config.n), m(config.m), p(config.p), F(config.F), H(config.H),
      Q(config.Q), R(config.R), P(config.P), S(config.n, config.n),
      K(config.p, config.n), I(config.n, config.n), x(config.n),
      y_hat(config.p), res(config.p), f(config.f), dfdx(config.dfdx),
      h(config.h), dhdx(config.dhdx)
{
    x.setZero();
    I.setIdentity();
}

void ExtendedKalmanEigen::init(const VectorXf& x0) { x = x0; }

void ExtendedKalmanEigen::predict(const VectorXf& u)
{
    F = dfdx(x, u);  // update jacobian

    P = F * P * F.transpose() + Q;
    x = f(x, u);
}

bool ExtendedKalmanEigen::correct(const VectorXf& y)
{
    return correct(y, this->h, this->dhdx);
}

bool ExtendedKalmanEigen::correct(const VectorXf& y, const function_v& h,
                                  const function_v& dhdx)
{
    H = dhdx(x);  // update jacobian

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

    x = x + K * (y - h(x));

    res = y - h(x);

    return true;
}

const VectorXf& ExtendedKalmanEigen::getState() { return x; }

const VectorXf& ExtendedKalmanEigen::getOutput()
{
    y_hat = h(x);
    return y_hat;
}

const VectorXf& ExtendedKalmanEigen::getResidual() { return res; }

void ExtendedKalmanEigen::setF(const MatrixXf& F) { this->F = F; }

void ExtendedKalmanEigen::setH(const MatrixXf& H) { this->H = H; }

void ExtendedKalmanEigen::setQ(const MatrixXf& Q) { this->Q = Q; }

void ExtendedKalmanEigen::setR(const MatrixXf& R) { this->R = R; }

void ExtendedKalmanEigen::setP(const MatrixXf& P) { this->P = P; }

void ExtendedKalmanEigen::setf(const function_2v& f) { this->f = f; }

void ExtendedKalmanEigen::setdfdx(const function_2v& dfdx)
{
    this->dfdx = dfdx;
}

void ExtendedKalmanEigen::seth(const function_v& h) { this->h = h; }

void ExtendedKalmanEigen::setdhdx(const function_v& dhdx) { this->dhdx = dhdx; }