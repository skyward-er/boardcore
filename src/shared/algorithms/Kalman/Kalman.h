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
 * @brief Implementation of a generic Kalman filter using the Eigen library.
 *
 * This class uses templates in order to know the size of each matrix at
 * compile-time. This way we avoid Eigen to allocate memory dynamically.
 */
template <typename T, int N_size, int P_size, int M_size = 1>
class Kalman
{
public:
    using MatrixNN = Eigen::Matrix<T, N_size, N_size>;
    using MatrixPN = Eigen::Matrix<T, P_size, N_size>;
    using MatrixNP = Eigen::Matrix<T, N_size, P_size>;
    using MatrixPP = Eigen::Matrix<T, P_size, P_size>;
    using MatrixNM = Eigen::Matrix<T, N_size, M_size>;
    using CVectorN = Eigen::Vector<T, N_size>;
    using CVectorP = Eigen::Vector<T, P_size>;
    using CVectorM = Eigen::Vector<T, M_size>;

    /**
     * @brief Configuration struct for the Kalman class.
     */
    struct KalmanConfig
    {
        MatrixNN F;
        MatrixPN H;
        MatrixNN Q;
        MatrixPP R;
        MatrixNN P;
        MatrixNM G;
        CVectorN x;
    };

    /**
     * @brief Creates a Kalman filter object.
     *
     * @param config Configuration parameters.
     */
    Kalman(const KalmanConfig& config)
        : F(config.F), H(config.H), Q(config.Q), R(config.R), P(config.P),
          G(config.G), S(MatrixPP::Zero(P_size, P_size)),
          K(MatrixNP::Zero(N_size, P_size)), x(config.x)
    {
        I.setIdentity();
    }

    void setConfig(const KalmanConfig& config)
    {
        F = config.F;
        H = config.H;
        Q = config.Q;
        R = config.R;
        P = config.P;
        G = config.G;
        S = MatrixPP::Zero(P_size, P_size);
        K = MatrixNP::Zero(N_size, P_size);
        x = config.x;
    }

    /**
     * @brief Prediction step with previous F matrix.
     */
    void predict()
    {
        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    /**
     * @brief Prediction step.
     *
     * @param F_new updated F matrix.
     */
    void predictNewF(const MatrixNN& F_new)
    {
        F = F_new;
        predict();
    }

    /**
     * @brief Prediction step with previous F matrix and with the control
     * vector.
     */
    void predictWithControl(const CVectorM& control)
    {
        x = F * x + G * control;
        P = F * P * F.transpose() + Q;
    }

    /**
     * @brief Prediction step.
     *
     * @param F_new updated F matrix.
     * @param control Control vector.
     */
    void predictWithControlNewF(const MatrixNN& F_new, const CVectorM& control)
    {
        F = F_new;
        predictWithControl(control);
    }

    /**
     * @brief Correction step.
     *
     * @param y The measurement vector.
     */
    bool correct(const CVectorP& y)
    {
        S = H * P * H.transpose() + R;

        // TODO: The determinant is computed here and when S is inverted, it
        // could be optimized
        if (S.determinant() < 1e-3)
            return false;

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
     * @brief Predicts k steps ahead the state.
     */
    const CVectorN predictState(uint32_t k)
    {
        CVectorN xHat = x;

        for (uint32_t i = 0; i < k; i++)
            xHat = F * xHat;

        return xHat;
    }

    /**
     * @brief Predicts k steps ahead the output.
     */
    const CVectorP predictOutput(uint32_t k) { return H * predictState(k); }

private:
    MatrixNN F; /**< State propagation matrix (n x n) */
    MatrixPN H; /**< Output matrix (p x n) */
    MatrixNN Q; /**< Model variance matrix (n x n) */
    MatrixPP R; /**< Measurement variance (p x p) */
    MatrixNN P; /**< Error covariance matrix (n x n) */
    MatrixNM G; /**< Input matrix (n x m) */

    MatrixPP S;
    MatrixNP K; /**< kalman gain */

    MatrixNN I; /**< identity matrix (n x n) */

    CVectorN x;    /**< state vector (n x 1) */
    CVectorP yHat; /**< output vector (p x 1) */
    CVectorP res;  /**< residual error vector (p x 1) */
};

}  // namespace Boardcore
