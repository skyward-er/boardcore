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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <Eigen/Dense>

namespace Boardcore
{

/**
 * @brief Class representing a Kalman filter using the Eigen library for
 *        matrix computations
 *
 * WARNING : This class uses templates in order to know the size of each matrix
 *           at compile-time. This way we avoid Eigen to allocate memory
 *           dynamically.
 */
template <typename t, uint8_t n, uint8_t p>
class KalmanEigen
{
    using MatrixNN = Eigen::Matrix<t, n, n>;
    using MatrixPN = Eigen::Matrix<t, p, n>;
    using MatrixNP = Eigen::Matrix<t, n, p>;
    using MatrixPP = Eigen::Matrix<t, p, p>;
    using CVectorN = Eigen::Matrix<t, n, 1>;
    using CVectorP = Eigen::Matrix<t, p, 1>;

public:
    /**
     * @brief Configuration struct for the KalmanEigen class.
     */
    struct KalmanConfig
    {
        MatrixNN F;
        MatrixPN H;
        MatrixNN Q;
        MatrixPP R;
        MatrixNN P;
        CVectorN x;
    };

    /**
     * @param config configuration object containing all the initialized
     *               matrices
     */
    KalmanEigen(const KalmanConfig& config)
        : F(config.F), H(config.H), Q(config.Q), R(config.R), P(config.P),
          S(MatrixPP::Zero(p, p)), K(MatrixNP::Zero(n, p)), x(config.x)
    {
        I.setIdentity();
    }

    /**
     * @brief Prediction step.
     */
    void predict()
    {
        P = F * P * F.transpose() + Q;
        x = F * x;
    }

    /**
     * @brief Prediction step.
     *
     * @param F_new updated F matrix
     */
    void predict(const MatrixNN& F_new)
    {
        this->F = F_new;
        this->predict();
    }

    /**
     * @brief Correction step (correct the estimate).
     *
     * @param y The measurement vector
     */
    bool correct(const CVectorP& y)
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

        res = y - H * x;

        return true;
    }

    /**
     * @return state vector
     */
    const CVectorN getState() { return x; }

    /**
     * @return output vector
     */
    const CVectorP getOutput()
    {
        yHat = H * x;
        return yHat;
    }

    /**
     * @return residual error vector
     */
    const CVectorP getResidual() { return res; }

    /**
     * @brief Predicts k steps ahead the output
     */
    const CVectorP predictOutput(uint32_t k) { return H * predictState(k); }

    /**
     * @brief Predicts k steps ahead the state
     */
    const CVectorN predictState(uint32_t k)
    {
        CVectorN xHat = x;

        for (uint32_t i = 0; i < k; i++)
        {
            xHat = F * xHat;
        }
        return xHat;
    }

private:
    MatrixNN F; /**< State propagation matrix (n x n) */
    MatrixPN H; /**< Output matrix (p x n) */
    MatrixNN Q; /**< Model variance matrix (n x n) */
    MatrixPP R; /**< Measurement variance (p x p) */
    MatrixNN P; /**< Error covariance matrix (n x n) */

    MatrixPP S;
    MatrixNP K; /**< kalman gain */

    MatrixNN I; /**< identity matrix (n x n) */

    CVectorN x;    /**< state vector (n x 1) */
    CVectorP yHat; /**< output vector (p x 1) */
    CVectorP res;  /**< residual error vector (p x 1) */
};

}  // namespace Boardcore
