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

#include <NavigationAttitudeSystem/ExtendedKalmanEigen.h>

using namespace Boardcore;
using namespace Eigen;

namespace DeathStackBoard
{

using namespace NASConfigs;

ExtendedKalmanEigen::ExtendedKalmanEigen()
{
    eye6.setIdentity();
    eye4.setIdentity();
    eye3.setIdentity();
    eye2.setIdentity();

    // clang-format off

    R_bar << SIGMA_BAR;
    R_mag << SIGMA_MAG * eye3;
    R_gps << eye4 * SIGMA_GPS / SATS_NUM;
    Q_mag << (SIGMA_W * SIGMA_W * T +
              (1.0F / 3.0F) * SIGMA_BETA * SIGMA_BETA * T * T * T) *
                 eye3,
        (0.5F * SIGMA_BETA * SIGMA_BETA * T * T) * eye3,
        // cppcheck-suppress constStatement
        (0.5F * SIGMA_BETA * SIGMA_BETA * T * T) * eye3,
        (SIGMA_BETA * SIGMA_BETA * T) * eye3;

    P_pos << P_POS, 0,     0,
             0,     P_POS, 0,
             0,     0,     P_POS_VERTICAL;
    P_vel << P_VEL, 0,     0,
             0,     P_VEL, 0,
             0,     0,     P_VEL_VERTICAL;
    P_att  = eye3 * P_ATT;
    P_bias = eye3 * P_BIAS;
    P << P_pos, Eigen::MatrixXf::Zero(3, N - 4), Eigen::MatrixXf::Zero(3, 3), P_vel,
        Eigen::MatrixXf::Zero(3, N - 7), Eigen::MatrixXf::Zero(3, 6), P_att,
        Eigen::MatrixXf::Zero(3, N - 10), Eigen::MatrixXf::Zero(3, 9), P_bias;

    Q_pos = eye3 * SIGMA_POS;
    Q_vel = eye3 * SIGMA_VEL;
    Q_lin << Q_pos, Eigen::MatrixXf::Zero(3, NL - 3), Eigen::MatrixXf::Zero(3, 3), Q_vel;

    F << 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
         0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
         0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F,
         Eigen::MatrixXf::Zero(3, NL);
    F   = eye6 + T * F;
    Ftr = F.transpose();

    H_gps << 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
             0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
             0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
             // cppcheck-suppress constStatement
             0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F;
    H_gpstr = H_gps.transpose();

    Fatt << -eye3, -eye3 * T, Eigen::Matrix3f::Zero(3, 3), eye3;
    Fatttr = Fatt.transpose();

    Gatt << -eye3, Eigen::Matrix3f::Zero(3, 3),
            Eigen::Matrix3f::Zero(3, 3), eye3;
    Gatttr = Gatt.transpose();

    g << 0.0F, 0.0F, aeroutils::constants::g; // [m/s^2]

    // clang-format on
}

void ExtendedKalmanEigen::predict(const Eigen::Vector3f& u)
{
    Eigen::Matrix3f A;
    Eigen::Vector3f a;
    VectorNf x_dot;

    Plin                  = P.block<NL, NL>(0, 0);
    P.block<NL, NL>(0, 0) = F * Plin * Ftr + Q_lin;

    float q1 =
        x(NL);  // Since the index begins from 0, the first quaternion
                // component will be at index  = # linear states before itself
    float q2 = x(NL + 1);
    float q3 = x(NL + 2);
    float q4 = x(NL + 3);

    A << powf(q1, 2) - powf(q2, 2) - powf(q3, 2) + powf(q4, 2),
        2.0F * q1 * q2 + 2.0F * q3 * q4, 2.0F * q1 * q3 - 2.0F * q2 * q4,
        2.0F * q1 * q2 - 2.0F * q3 * q4,
        -powf(q1, 2) + powf(q2, 2) - powf(q3, 2) + powf(q4, 2),
        2.0F * q2 * q3 + 2.0F * q1 * q4, 2.0F * q1 * q3 + 2.0F * q2 * q4,
        // cppcheck-suppress constStatement
        2.0F * q2 * q3 - 2.0F * q1 * q4,
        -powf(q1, 2) - powf(q2, 2) + powf(q3, 2) + powf(q4, 2);

    a = A.transpose() * u +
        g;  // Real accelerometers don't measure g during the flight

    x_dot << x(3), x(4), x(5), a,
        Eigen::MatrixXf::Zero(
            NATT,
            1);  // The quaternions and the biases don't need to be updated

    x = x + T * x_dot;
}

void ExtendedKalmanEigen::correctBaro(const float y, const float msl_press,
                                      const float msl_temp)
{
    Matrix<float, NL, NBAR> K_bar;
    Matrix<float, NBAR, NL> H_bar;
    Matrix<float, NBAR, NBAR> S_bar;

    Plin = P.block<NL, NL>(0, 0);

    // temperature at current altitude :
    // since x(2) (altitude) is negative, mslTemperature returns temperature at
    // current altitude and not at mean sea level
    float temp = aeroutils::mslTemperature(msl_temp, x(2));

    // compute gradient of the altitude-pressure function
    float dp_dx = aeroutils::constants::a * aeroutils::constants::n *
                  msl_press *
                  powf(1 - aeroutils::constants::a * x(2) / temp,
                       -aeroutils::constants::n - 1) /
                  temp;

    H_bar << 0.0F, 0.0F, dp_dx,
        Eigen::MatrixXf::Zero(1, N - 3 - NATT);  // Gradient of h_bar

    S_bar = H_bar * Plin * H_bar.transpose() + R_bar;

    K_bar = Plin * H_bar.transpose() * S_bar.inverse();

    P.block<NL, NL>(0, 0) = (eye6 - K_bar * H_bar) * Plin;

    float y_hat = aeroutils::mslPressure(msl_press, msl_temp, x(2));

    x.head(NL) = x.head(NL) + K_bar * (y - y_hat);

    // float res_bar = y - h_bar;
}

void ExtendedKalmanEigen::correctGPS(const Vector4f& y, const uint8_t sats_num)
{
    Matrix<float, NGPS, 1> h_gps;
    Matrix<float, NL, NGPS> K_gps;
    Matrix<float, NGPS, 1> res_gps;
    Matrix<float, NGPS, NGPS> S_gps;

    float xnord   = y(0);
    float yest    = y(1);
    float velnord = y(2);
    float velest  = y(3);

    Matrix<float, NGPS, 1> yned(xnord, yest, velnord, velest);

    R_gps = eye4 * SIGMA_GPS / sqrtf(sats_num);

    Plin = P.block<NL, NL>(0, 0);

    S_gps = H_gps * Plin * H_gpstr + R_gps;

    K_gps = Plin * H_gpstr * S_gps.inverse();

    P.block<NL, NL>(0, 0) = (eye6 - K_gps * H_gps) * Plin;

    h_gps << x(0), x(1), x(3), x(4);

    x.head(NL) = x.head(NL) + K_gps * (yned - h_gps);

    // cppcheck-suppress unreadVariable
    res_gps = y - h_gps;
}

const VectorNf& ExtendedKalmanEigen::getState() { return x; }

void ExtendedKalmanEigen::setX(const VectorNf& x) { this->x = x; }

/*
    MULTIPLICATIVE EXTENDED KALMAN FILTER
*/
void ExtendedKalmanEigen::predictMEKF(const Eigen::Vector3f& u)
{
    Eigen::Vector3f omega;
    Eigen::Vector3f prev_bias;
    Matrix4f omega_mat;
    Eigen::Matrix3f omega_submat;

    q << x(NL), x(NL + 1), x(NL + 2), x(NL + 3);
    prev_bias << x(NL + 4), x(NL + 5), x(NL + 6);

    omega = u - prev_bias;

    omega_submat << 0.0F, -omega(2), omega(1), omega(2), 0.0F, -omega(0),
        // cppcheck-suppress constStatement
        -omega(1), omega(0), 0.0F;

    // cppcheck-suppress constStatement
    omega_mat << -omega_submat, omega, -omega.transpose(), 0.0F;

    q = (eye4 + 0.5F * omega_mat * T) * q;
    q.normalize();

    // cppcheck-suppress constStatement
    x.tail(NATT) << q, prev_bias;

    Patt = P.block<NMEKF, NMEKF>(
        NL, NL);  // The block of P related to the attitude starts at (NL, NL)
                  // because the index starts from 0. The block is of size
                  // NMEKF x NMEKF.
    P.block<NMEKF, NMEKF>(NL, NL) =
        Fatt * Patt * Fatttr +
        Gatt * Q_mag * Gatttr;  // Update only the attitude related part of P
}

void ExtendedKalmanEigen::correctMEKF(const Eigen::Vector3f& y)
{
    Eigen::Matrix3f A;
    Eigen::Vector3f z;
    Vector4f r;
    Eigen::Matrix3f z_mat;
    Vector4f u_att;
    Eigen::Vector3f r_sub;
    Matrix<float, NMAG, NMAG> Satt;
    Matrix<float, NMAG, NMEKF> Hatt;
    Matrix<float, NMEKF, NMAG> Katt;
    Matrix<float, NMEKF, 1> delta_x;

    float q1 = x(NL);
    float q2 = x(NL + 1);
    float q3 = x(NL + 2);
    float q4 = x(NL + 3);

    A << powf(q1, 2) - powf(q2, 2) - powf(q3, 2) + powf(q4, 2),
        2.0F * q1 * q2 + 2.0F * q3 * q4, 2.0F * q1 * q3 - 2.0F * q2 * q4,
        2.0F * q1 * q2 - 2.0F * q3 * q4,
        -powf(q1, 2) + powf(q2, 2) - powf(q3, 2) + powf(q4, 2),
        2.0F * q2 * q3 + 2.0F * q1 * q4, 2.0F * q1 * q3 + 2.0F * q2 * q4,
        // cppcheck-suppress constStatement
        2.0F * q2 * q3 - 2.0F * q1 * q4,
        -powf(q1, 2) - powf(q2, 2) + powf(q3, 2) + powf(q4, 2);

    z = A * NED_MAG;

    // cppcheck-suppress constStatement
    z_mat << 0.0F, -z(2), z(1), z(2), 0.0F, -z(0), -z(1), z(0), 0.0F;

    Hatt << z_mat, Eigen::Matrix3f::Zero(3, 3);

    Patt = P.block<NMEKF, NMEKF>(NL, NL);
    Satt = Hatt * Patt * Hatt.transpose() + R_mag;

    Katt = Patt * Hatt.transpose() * Satt.inverse();

    delta_x = Katt * (y - z);

    r_sub << delta_x(0), delta_x(1), delta_x(2);

    r << 0.5F * r_sub, sqrtf(1.0F - 0.25F * r_sub.transpose() * r_sub);

    // cppcheck-suppress constStatement
    q << q1, q2, q3, q4;
    u_att        = quater.quatProd(r, q);
    float u_norm = u_att.norm();

    x(NL)     = u_att(0) / u_norm;
    x(NL + 1) = u_att(1) / u_norm;
    x(NL + 2) = u_att(2) / u_norm;
    x(NL + 3) = u_att(3) / u_norm;
    x(NL + 4) = x(NL + 4) + delta_x(3);
    x(NL + 5) = x(NL + 5) + delta_x(4);
    x(NL + 6) = x(NL + 6) + delta_x(5);

    P.block<NMEKF, NMEKF>(NL, NL) =
        (eye6 - Katt * Hatt) * Patt * (eye6 - Katt * Hatt).transpose() +
        Katt * R_mag * Katt.transpose();
}

}  // namespace DeathStackBoard
