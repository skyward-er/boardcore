/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Alessandro Del Duca, Luca Conterio, Marco Cella
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

#include "ExtendedKalman.h"

#include <utils/SkyQuaternion/SkyQuaternion.h>

using namespace Boardcore;
using namespace Eigen;

namespace Boardcore
{

ExtendedKalman::ExtendedKalman(ExtendedKalmanConfig config) : config(config)
{
    // Initialize magnetometer utility matix
    R_mag << config.SIGMA_MAG * Matrix3f::Identity();

    // Initialize GPS utility matixes
    H_gps                = Matrix<float, 4, 6>::Identity();
    H_gps.coeffRef(2, 2) = 0;
    H_gps.coeffRef(5, 5) = 0;
    H_gps_tr             = H_gps.transpose();
    R_gps << config.SIGMA_GPS * Matrix<float, 4, 4>::Identity();

    // TODO: Review

    Q_mag << (config.SIGMA_W * config.SIGMA_W * config.T +
              (1.0f / 3.0f) * config.SIGMA_BETA * config.SIGMA_BETA * config.T *
                  config.T * config.T) *
                 Matrix3f::Identity(),
        (0.5F * config.SIGMA_BETA * config.SIGMA_BETA * config.T * config.T) *
            Matrix3f::Identity(),
        (0.5F * config.SIGMA_BETA * config.SIGMA_BETA * config.T * config.T) *
            // cppcheck-suppress constStatement
            Matrix3f::Identity(),
        (config.SIGMA_BETA * config.SIGMA_BETA * config.T) *
            Matrix3f::Identity();

    Eigen::Matrix3f P_pos{{config.P_POS, 0, 0},
                          {0, config.P_POS, 0},
                          {0, 0, config.P_POS_VERTICAL}};
    Eigen::Matrix3f P_vel{{config.P_VEL, 0, 0},
                          {0, config.P_VEL, 0},
                          {0, 0, config.P_VEL_VERTICAL}};
    Eigen::Matrix3f P_att  = Matrix3f::Identity() * config.P_ATT;
    Eigen::Matrix3f P_bias = Matrix3f::Identity() * config.P_BIAS;
    P << P_pos, MatrixXf::Zero(3, 13 - 4), MatrixXf::Zero(3, 3), P_vel,
        MatrixXf::Zero(3, 13 - 7), MatrixXf::Zero(3, 6), P_att,
        // cppcheck-suppress constStatement
        MatrixXf::Zero(3, 13 - 10), MatrixXf::Zero(3, 9), P_bias;

    Eigen::Matrix3f Q_pos = Matrix3f::Identity() * config.SIGMA_POS;
    Eigen::Matrix3f Q_vel = Matrix3f::Identity() * config.SIGMA_VEL;
    // cppcheck-suppress constStatement
    Q_lin << Q_pos, MatrixXf::Zero(3, 6 - 3), MatrixXf::Zero(3, 3), Q_vel;

    F << 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, MatrixXf::Zero(3, 6);
    F   = Matrix<float, 6, 6>::Identity() + config.T * F;
    Ftr = F.transpose();

    Fatt << -Matrix3f::Identity(), -Matrix3f::Identity() * config.T,
        Matrix3f::Zero(3, 3), Matrix3f::Identity();
    Fatttr = Fatt.transpose();

    Gatt << -Matrix3f::Identity(), Matrix3f::Zero(3, 3), Matrix3f::Zero(3, 3),
        Matrix3f::Identity();
    Gatttr = Gatt.transpose();

    gravityNed << 0.0f, 0.0f, Constants::g;  // [m/s^2]
}

void ExtendedKalman::predictAcc(const Vector3f& acceleration)
{
    Matrix3f A   = body2ned(x.block<4, 1>(IDX_QUAT, 0));
    Vector3f pos = x.block<3, 1>(IDX_POS, 0);
    Vector3f vel = x.block<3, 1>(IDX_VEL, 0);

    // Update position by integrating the velocity
    pos = pos + vel * config.T;

    // Measured acceleration in NED frame with added gravity
    Vector3f a = A.transpose() * acceleration + gravityNed;

    // Update velocity by integrating the acceleration
    vel = vel + a * config.T;

    // Save the updated state
    x.block<3, 1>(IDX_POS, 0) = pos;
    x.block<3, 1>(IDX_VEL, 0) = vel;

    // Variance propagation
    // TODO: Review
    Eigen::Matrix<float, 6, 6> Plin = P.block<6, 6>(0, 0);
    P.block<6, 6>(0, 0)             = F * Plin * Ftr + Q_lin;
}

void ExtendedKalman::predictGyro(const Vector3f& angularVelocity)
{
    Vector3f bias = x.block<3, 1>(IDX_BIAS, 0);
    Vector4f q    = x.block<4, 1>(IDX_QUAT, 0);

    Vector3f omega = angularVelocity - bias;
    // clang-format off
    Matrix4f omega_mat{
        { 0.0f,      omega(2), -omega(1), omega(0)},
        {-omega(2),  0.0f,      omega(0), omega(1)},
        { omega(1), -omega(0),  0.0f,     omega(2)},
        {-omega(0), -omega(1), -omega(2), 0.0f}
    };
    // clang-format on

    // Update orientation by integrating the angular velocity
    q = (Matrix4f::Identity() + 0.5F * omega_mat * config.T) * q;
    q.normalize();

    // Save the updated state
    x.block<4, 1>(IDX_QUAT, 0) = q;

    // Variance propagation
    // TODO: Review
    Patt = P.block<6, 6>(IDX_QUAT, IDX_QUAT);
    P.block<6, 6>(IDX_QUAT, IDX_QUAT) =
        Fatt * Patt * Fatttr + Gatt * Q_mag * Gatttr;
}

void ExtendedKalman::correctBaro(const float pressure, const float mslPress,
                                 const float mslTemp)
{
    Matrix<float, 1, 6> H = Matrix<float, 1, 6>::Zero();

    // Temperature at current altitude. Since in NED the altitude is negative,
    // mslTemperature returns temperature at current altitude and not at msl
    float temp = Aeroutils::mslTemperature(mslTemp, x(2));

    // Compute gradient of the altitude-pressure function
    H[2] = Constants::a * Constants::n * mslPress *
           powf(1 - Constants::a * x(2) / temp, -Constants::n - 1) / temp;

    Eigen::Matrix<float, 6, 6> Pl = P.block<6, 6>(0, 0);
    Matrix<float, 1, 1> S =
        H * Pl * H.transpose() + Matrix<float, 1, 1>(config.SIGMA_BAR);
    Matrix<float, 6, 1> K = Pl * H.transpose() * S.inverse();
    P.block<6, 6>(0, 0)   = (Matrix<float, 6, 6>::Identity() - K * H) * Pl;

    float y_hat = Aeroutils::mslPressure(mslPress, mslTemp, x(2));

    // Update the state
    x.head<6>() = x.head<6>() + K * (pressure - y_hat);
}

void ExtendedKalman::correctGPS(const Vector4f& gps, const uint8_t sats_num)
{
    Eigen::Matrix<float, 6, 6> Pl = P.block<6, 6>(0, 0);

    Matrix<float, 4, 4> S = H_gps * Pl * H_gps_tr + R_gps;
    Matrix<float, 6, 4> K = Pl * H_gps_tr * S.inverse();

    P.block<6, 6>(0, 0) =
        (Eigen::Matrix<float, 6, 6>::Identity() - K * H_gps) * Pl;

    // Current state [n e vn ve]
    Matrix<float, 4, 1> H{x(0), x(1), x(3), x(4)};

    // Update the state
    x.head<6>() = x.head<6>() + K * (gps - H);
}

void ExtendedKalman::correctMag(const Vector3f& mag)
{
    Vector4f q = x.block<4, 1>(IDX_QUAT, 0);
    Matrix3f A = body2ned(q);

    Vector3f h;  // Magnetormeter in NED
    Vector4f r;
    Vector4f u_att;
    Vector3f r_sub;
    Matrix<float, 3, 3> Satt;
    Matrix<float, 3, 6> Hatt;
    Matrix<float, 6, 3> Katt;
    Matrix<float, 6, 1> delta_x;

    h = A.transpose() * mag;

    Vector3f r_mag{static_cast<float>(sqrt(h(0) * h(0) + h(1) * h(1))), 0,
                   h(2)};
    Vector3f mEst = A * r_mag;

    Matrix3f M{
        {0, -mEst(2), mEst(1)}, {mEst(2), 0, -mEst(0)}, {-mEst(1), mEst(0), 0}};

    Hatt << M, Matrix3f::Zero(3, 3);

    Patt = P.block<6, 6>(6, 6);
    Satt = Hatt * Patt * Hatt.transpose() + R_mag;

    Katt = Patt * Hatt.transpose() * Satt.inverse();

    delta_x = Katt * (mag - mEst);

    r_sub << delta_x(0), delta_x(1), delta_x(2);

    r << 0.5F * r_sub, sqrtf(1.0f - 0.25F * r_sub.transpose() * r_sub);

    u_att        = SkyQuaternion::quatProd(r, q);
    float u_norm = u_att.norm();

    x(6)     = u_att(0) / u_norm;
    x(6 + 1) = u_att(1) / u_norm;
    x(6 + 2) = u_att(2) / u_norm;
    x(6 + 3) = u_att(3) / u_norm;
    x(6 + 4) = x(6 + 4) + delta_x(3);
    x(6 + 5) = x(6 + 5) + delta_x(4);
    x(6 + 6) = x(6 + 6) + delta_x(5);

    P.block<6, 6>(6, 6) =
        (Matrix<float, 6, 6>::Identity() - Katt * Hatt) * Patt *
            (Matrix<float, 6, 6>::Identity() - Katt * Hatt).transpose() +
        Katt * R_mag * Katt.transpose();
}

Matrix<float, 13, 1> ExtendedKalman::getState() const { return x; }

void ExtendedKalman::setX(const Matrix<float, 13, 1>& x) { this->x = x; }

Matrix3f ExtendedKalman::body2ned(const Vector4f& q)
{
    // clang-format off
    return Matrix3f{
        {
            powf(q[0], 2) - powf(q[1], 2) - powf(q[2], 2) + powf(q[3], 2),
            2.0f * (q[0] * q[1] + q[2] * q[3]),
            2.0f * (q[0] * q[2] - q[1] * q[3]),
        },
        {
            2.0f * (q[0] * q[1] - q[2] * q[3]),
            -powf(q[0], 2) + powf(q[1], 2) - powf(q[2], 2) + powf(q[3], 2),
            2.0f * (q[1] * q[2] + q[0] * q[3]),
        },
        {
            2.0f * (q[0] * q[2] + q[1] * q[3]),
            2.0f * (q[1] * q[2] - q[0] * q[3]),
            -powf(q[0], 2) - powf(q[1], 2) + powf(q[2], 2) + powf(q[3], 2)
        }
    };
    // clanf-format on
}

}  // namespace Boardcore
