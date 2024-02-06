/* Copyright (c) 2020-2022 Skyward Experimental Rocketry
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

#include "NAS.h"

#include <drivers/timer/TimestampTimer.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <iostream>

using namespace Boardcore;
using namespace Eigen;

namespace Boardcore
{

NAS::NAS(NASConfig config) : config(config)
{
    // Covariance setup
    {
        // clang-format off
        Matrix3f P_pos{
            {config.P_POS, 0,            0},
            {0,            config.P_POS, 0},
            {0,            0,            config.P_POS_VERTICAL}
        };
        Matrix3f P_vel{
            {config.P_VEL, 0,            0},
            {0,            config.P_VEL, 0},
            {0,            0,            config.P_VEL_VERTICAL}
        };
        Matrix3f P_att  = Matrix3f::Identity() * config.P_ATT;
        Matrix3f P_bias = Matrix3f::Identity() * config.P_BIAS;
        P << P_pos,               MatrixXf::Zero(3, 9),
            MatrixXf::Zero(3, 3), P_vel, MatrixXf::Zero(3, 6),
            MatrixXf::Zero(3, 6),        P_att, MatrixXf::Zero(3, 3),
            MatrixXf::Zero(3, 9),               P_bias; // cppcheck-suppress constStatement
        // clang-format on
    }

    // GPS
    {
        H_gps                = Matrix<float, 4, 6>::Zero();
        H_gps.coeffRef(0, 0) = 1;
        H_gps.coeffRef(1, 1) = 1;
        H_gps.coeffRef(2, 3) = 1;
        H_gps.coeffRef(3, 4) = 1;
        H_gps_tr             = H_gps.transpose();
        R_gps << config.SIGMA_GPS * Matrix<float, 4, 4>::Identity();
    }

    // Utility matrixes
    R_acc << config.SIGMA_ACC * Matrix3f::Identity();
    R_mag << config.SIGMA_MAG * Matrix3f::Identity();
    {
        // clang-format off
        Q_quat << (config.SIGMA_W * config.SIGMA_W * config.T + (1.0f / 3.0f) * config.SIGMA_BETA * config.SIGMA_BETA * config.T * config.T * config.T) * Matrix3f::Identity(),
            (0.5f * config.SIGMA_BETA * config.SIGMA_BETA * config.T * config.T) * Matrix3f::Identity(),
            (0.5f * config.SIGMA_BETA * config.SIGMA_BETA * config.T * config.T) * Matrix3f::Identity(), // cppcheck-suppress constStatement
            (config.SIGMA_BETA * config.SIGMA_BETA * config.T) * Matrix3f::Identity();
        // clang-format on

        Q_lin << config.SIGMA_POS * Matrix3f::Identity(), MatrixXf::Zero(3, 3),
            // cppcheck-suppress constStatement
            MatrixXf::Zero(3, 3), config.SIGMA_VEL * Matrix3f::Identity();

        F_lin                   = Matrix<float, 6, 6>::Identity();
        F_lin.block<3, 3>(0, 3) = config.T * Matrix3f::Identity();
        F_lin_tr                = F_lin.transpose();

        F_quat << -Matrix3f::Identity(), -Matrix3f::Identity() * config.T,
            Matrix3f::Zero(3, 3), Matrix3f::Identity();
        F_quat_tr = F_quat.transpose();

        G_quat << -Matrix3f::Identity(), Matrix3f::Zero(3, 3),
            Matrix3f::Zero(3, 3), Matrix3f::Identity();
        G_quat_tr = G_quat.transpose();
    }
}

void NAS::predictAcc(const Vector3f& acceleration)
{
    Matrix3f A = SkyQuaternion::quat2rotationMatrix(x.block<4, 1>(IDX_QUAT, 0));
    Vector3f pos = x.block<3, 1>(IDX_POS, 0);
    Vector3f vel = x.block<3, 1>(IDX_VEL, 0);

    // Update position by integrating the velocity
    pos = pos + vel * config.T;

    // Measured acceleration in NED frame without gravity
    Vector3f a = A * acceleration - gravityNed;

    // Update velocity by integrating the acceleration
    vel = vel + a * config.T;

    // Save the updated state
    x.block<3, 1>(IDX_POS, 0) = pos;
    x.block<3, 1>(IDX_VEL, 0) = vel;

    // Variance propagation
    Matrix<float, 6, 6> Pl = P.block<6, 6>(0, 0);
    P.block<6, 6>(0, 0)    = F_lin * Pl * F_lin_tr + Q_lin;
}

void NAS::predictAcc(const AccelerometerData& acceleration)
{
    predictAcc(Vector3f{acceleration.accelerationX.value(),
                        acceleration.accelerationY.value(),
                        acceleration.accelerationZ.value()});
}

void NAS::predictGyro(const Vector3f& angularSpeed)
{
    Vector3f bias = x.block<3, 1>(IDX_BIAS, 0);
    Vector4f q    = x.block<4, 1>(IDX_QUAT, 0);

    Vector3f omega = angularSpeed - bias;
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
    // TODO: Optimize last G_quat * Q_quat * G_att_tr
    Matrix<float, 6, 6> Pq = P.block<6, 6>(IDX_QUAT, IDX_QUAT);
    Pq = F_quat * Pq * F_quat_tr + G_quat * Q_quat * G_quat_tr;
    P.block<6, 6>(IDX_QUAT, IDX_QUAT) = Pq;
}

void NAS::predictGyro(const GyroscopeData& angularSpeed)
{
    predictGyro(Vector3f{angularSpeed.angularSpeedX, angularSpeed.angularSpeedY,
                         angularSpeed.angularSpeedZ});
}

void NAS::correctBaro(const float pressure)
{
    Matrix<float, 1, 6> H = Matrix<float, 1, 6>::Zero();

    // Temperature at current altitude. Since in NED the altitude is negative,
    // mslTemperature returns temperature at current altitude and not at msl
    float temp = Aeroutils::relTemperature(-x(2), reference.refTemperature);

    // Compute gradient of the altitude-pressure function
    H[2] = Constants::a * Constants::n * reference.refPressure *
           powf(1 - Constants::a * x(2) / temp, -Constants::n - 1) / temp;

    Matrix<float, 6, 6> Pl = P.block<6, 6>(0, 0);
    Matrix<float, 1, 1> S =
        H * Pl * H.transpose() + Matrix<float, 1, 1>(config.SIGMA_BAR);
    Matrix<float, 6, 1> K = Pl * H.transpose() * S.inverse();
    P.block<6, 6>(0, 0)   = (Matrix<float, 6, 6>::Identity() - K * H) * Pl;

    float y_hat =
        Aeroutils::relPressure(reference.refAltitude - x(2),
                               reference.mslPressure, reference.mslTemperature);

    // Update the state
    x.head<6>() = x.head<6>() + K * (pressure - y_hat);
}

void NAS::correctGPS(const Vector4f& gps)
{
    Matrix<float, 6, 6> Pl = P.block<6, 6>(0, 0);

    Matrix<float, 4, 4> S = H_gps * Pl * H_gps_tr + R_gps;
    Matrix<float, 6, 4> K = Pl * H_gps_tr * S.inverse();

    P.block<6, 6>(0, 0) = (Matrix<float, 6, 6>::Identity() - K * H_gps) * Pl;

    // Current state [n e vn ve]
    Matrix<float, 4, 1> H{x(0), x(1), x(3), x(4)};

    // Update the state
    x.head<6>() = x.head<6>() + K * (gps - H);
}

void NAS::correctGPS(const GPSData& gps)
{
    if (!gps.fix)
        return;

    auto gpsPos = Aeroutils::geodetic2NED(
        {gps.latitude, gps.longitude},
        {reference.refLatitude, reference.refLongitude});

    correctGPS(
        Vector4f{gpsPos(0), gpsPos(1), gps.velocityNorth, gps.velocityEast});
}

void NAS::correctMag(const Vector3f& mag)
{
    Vector4f q = x.block<4, 1>(IDX_QUAT, 0);
    Matrix3f A = SkyQuaternion::quat2rotationMatrix(q).transpose();

    // Rotate the NED magnetic field in the relative reference frame
    Vector3f mEst = A * config.NED_MAG;

    // clang-format off
    Matrix3f M{
        { 0,      -mEst(2), mEst(1)},
        { mEst(2), 0,      -mEst(0)},
        {-mEst(1), mEst(0), 0}
    };
    // clang-format on

    Matrix<float, 3, 6> H;
    H << M, Matrix3f::Zero(3, 3);
    Matrix<float, 6, 6> Pq = P.block<6, 6>(IDX_QUAT, IDX_QUAT);
    Matrix<float, 3, 3> S  = H * Pq * H.transpose() + R_mag;

    Matrix<float, 6, 3> K  = Pq * H.transpose() * S.inverse();
    Matrix<float, 6, 1> dx = K * (mag - mEst);
    Vector4f r{0.5f * dx(0), 0.5f * dx(1), 0.5f * dx(2),
               sqrtf(1.0f - 0.25f * dx.head<3>().squaredNorm())};

    x.block<4, 1>(IDX_QUAT, 0) = SkyQuaternion::quatProd(r, q);
    x.block<3, 1>(IDX_BIAS, 0) += dx.tail<3>();
    Matrix<float, 6, 6> tmp = Matrix<float, 6, 6>::Identity() - K * H;
    Pq = tmp * Pq * tmp.transpose() + K * R_mag * K.transpose();
    P.block<6, 6>(IDX_QUAT, IDX_QUAT) = Pq;
}

void NAS::correctMag(const MagnetometerData& mag)
{
    Vector3f magV = {mag.magneticFieldX, mag.magneticFieldY,
                     mag.magneticFieldZ};
    correctMag(magV.normalized());
}

void NAS::correctAcc(const Vector3f& acc)
{
    Vector4f q = x.block<4, 1>(IDX_QUAT, 0);
    Matrix3f A = SkyQuaternion::quat2rotationMatrix(q).transpose();

    // Rotate the NED magnetic field in the relative reference frame
    Vector3f aEst = A * gravityNed;
    aEst.normalize();

    // Gradient matrix
    // clang-format off
    Matrix3f M{
        { 0,      -aEst(2), aEst(1)},
        { aEst(2), 0,      -aEst(0)},
        {-aEst(1), aEst(0), 0}
    };
    // clang-format on

    Matrix<float, 3, 6> H;
    H << M, Matrix3f::Zero(3, 3);
    Matrix<float, 6, 6> Pq = P.block<6, 6>(IDX_QUAT, IDX_QUAT);
    // TODO: Change R_mag with R_acc
    Matrix<float, 3, 3> S = H * Pq * H.transpose() + R_mag;

    Matrix<float, 6, 3> K  = Pq * H.transpose() * S.inverse();
    Matrix<float, 6, 1> dx = K * (acc - aEst);
    Vector4f r{0.5f * dx(0), 0.5f * dx(1), 0.5f * dx(2),
               sqrtf(1.0f - 0.25f * dx.head<3>().squaredNorm())};

    x.block<4, 1>(IDX_QUAT, 0) = SkyQuaternion::quatProd(r, q);
    x.block<3, 1>(IDX_BIAS, 0) += dx.tail<3>();
    Matrix<float, 6, 6> tmp = Matrix<float, 6, 6>::Identity() - K * H;
    Pq = tmp * Pq * tmp.transpose() + K * R_mag * K.transpose();
    P.block<6, 6>(IDX_QUAT, IDX_QUAT) = Pq;
}

void NAS::correctAcc(const AccelerometerData& acc)
{
    Vector3f accV = {acc.accelerationX.value(), acc.accelerationY.value(),
                     acc.accelerationZ.value()};
    correctAcc(accV.normalized());
}

void NAS::correctPitot(const float airspeed)
{
    // Correct with pitot airspeed only if the airspeed is greater than zero. Do
    // not correct otherwise
    if (airspeed > 0)
    {
        Matrix<float, 1, 1> R(config.SIGMA_PITOT);
        // Take the already estimated velocities
        Vector3f vel = x.block<3, 1>(IDX_VEL, 0);

        Matrix<float, 1, 3> H = Matrix<float, 1, 3>::Zero();
        H.coeffRef(0, 2)      = -x[5] / airspeed;

        // Take covariance matrix for the velocities
        Matrix<float, 3, 3> Pl = P.block<3, 3>(IDX_VEL, IDX_VEL);
        Matrix<float, 1, 1> S  = H * Pl * H.transpose() + R;

        // Correct with the pitot only if S is invertible
        if (S[0] != 0)
        {
            // Compute the kalman Gain
            Matrix<float, 3, 1> K = Pl * H.transpose() * S.inverse();

            x.block<3, 1>(IDX_VEL, 0) = vel + K * (-airspeed - x[5]);

            P.block<3, 3>(IDX_VEL, IDX_VEL) =
                (Matrix<float, 3, 3>::Identity() - K * H) * Pl;
        }
    }
}

NASState NAS::getState() const
{
    return NASState(TimestampTimer::getTimestamp(), x);
}

Matrix<float, 13, 1> NAS::getX() const { return x; }

void NAS::setState(const NASState& state) { this->x = state.getX(); }

void NAS::setX(const Matrix<float, 13, 1>& x) { this->x = x; }

void NAS::setReferenceValues(const ReferenceValues reference)
{
    this->reference = reference;
}

ReferenceValues NAS::getReferenceValues() { return reference; }

}  // namespace Boardcore
