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

#pragma once

#include <Eigen/Dense>

#include "KalmanConfig.h"

using namespace Eigen;

class KalmanEigen
{

public:
    KalmanEigen(const KalmanConfig& config);

    void init(const VectorXf& x0);

    void predict();

    void predict(const MatrixXf& F_new);

    void predict(const VectorXf& u);

    void predict(const VectorXf& u, const MatrixXf& F_new);

    bool correct(const VectorXf& y);

    const VectorXf& getState();

    const VectorXf& getOutput();

    const VectorXf& getResidual();

    const VectorXf predictOutput(uint32_t k);

    const VectorXf predictState(uint32_t k);

private:
    uint8_t n, m, p; /**< system dimensions */

    MatrixXf F, G, H, Q, R, P, S, K;

    MatrixXf I; /**< (n x n) identity matrix */
    VectorXf x, y_hat, res;
};