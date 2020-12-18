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

class ExtendedKalmanEigen
{

public:
    ExtendedKalmanEigen(const ExtendedKalmanConfig& config);

    void predict(const VectorXf& u);

    /**
     * @brief Correct using the default h and dhdx
     *        (those contained in config)
     */
    bool correct(const VectorXf& y);

    /**
     * @brief Correct using the specified h and dhdx parameters
     */
    bool correct(const VectorXf& y, const function_v& h,
                 const function_v& dhdx);

    const VectorXf& getState();

    const VectorXf& getOutput();

    const VectorXf& getResidual();

    void setF(const MatrixXf& F);

    void setH(const MatrixXf& H);

    void setQ(const MatrixXf& Q);

    void setR(const MatrixXf& R);

    void setP(const MatrixXf& P);

    void setf(const function_2v& f);

    void setdfdx(const function_2v& dfdx);

    void seth(const function_v& h);

    void setdhdx(const function_v& dhdx);

private:
    uint8_t n, m, p; /**< system dimensions */

    MatrixXf F, H, Q, R, P;

    MatrixXf I; /**< (n x n) identity matrix */
    VectorXf x, y_hat, res;

    function_2v f;
    function_2v dfdx;
    function_v h;
    function_v dhdx;
};