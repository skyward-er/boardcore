/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Giovanni Annaloro
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

#include "ZVK.h"

#include <drivers/timer/TimestampTimer.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/MatrixFunctions>

#include "ZVKState.h"

using namespace Boardcore;
namespace Boardcore
{

ZVK::ZVK(const ZVKConfig& config) : config(config)
{
    // Covariance matrices setup

    // State covariance matrix setup
    {
        Eigen::Matrix<float, 15, 1> diagElements;
        // Set elements in the vector
        diagElements << Eigen::Vector3f::Ones() * smallAngleError *
                            smallAngleError,
            Eigen::Vector3f::Ones() * config.VEL_UNCERTAINTY,
            Eigen::Vector3f::Ones() * config.POS_UNCERTAINTY,
            Eigen::Vector3f::Ones() * config.BIAS_ACC,
            Eigen::Vector3f::Ones() * config.BIAS_GYRO;
        P = diagElements.asDiagonal();
    }

    // Measurement noise covariance matrix setup
    {
        R.setZero();
        // Set diagonal 3x3 blocks
        R.block<3, 3>(0, 0) = rConst * Eigen::Matrix3f::Identity();
        R.block<3, 3>(3, 3) = rConst * Eigen::Matrix3f::Identity();
        R.block<3, 3>(6, 6) =
            config.SIGMA_ACC * config.SIGMA_ACC * Eigen::Matrix3f::Identity();
        R.block<3, 3>(9, 9) =
            config.SIGMA_MAG * config.SIGMA_MAG * Eigen::Matrix3f::Identity();
    }

    // Process noise covariance matrix setup
    {
        Eigen::Matrix<float, 12, 1> diagElements;
        // Set elements in the vector
        diagElements << Eigen::Vector3f::Ones() * config.SIGMA_GYRO *
                            config.SIGMA_GYRO,
            Eigen::Vector3f::Ones() * config.SIGMA_GYRO_BIAS *
                config.SIGMA_GYRO_BIAS,
            Eigen::Vector3f::Ones() * config.SIGMA_ACC * config.SIGMA_ACC,
            Eigen::Vector3f::Ones() * config.SIGMA_GYRO_BIAS *
                config.SIGMA_GYRO_BIAS;
        Q = diagElements.asDiagonal();
    }
}

void ZVK::predict(const Eigen::Vector3f& acceleration,
                  const Eigen::Vector3f& angularSpeed)
{
    Eigen::Vector4f quat     = x.block<4, 1>(IDX_QUAT, 0);
    Eigen::Vector3f vel      = x.block<3, 1>(IDX_VEL, 0);
    Eigen::Vector3f pos      = x.block<3, 1>(IDX_POS, 0);
    Eigen::Vector3f accBias  = x.block<3, 1>(IDX_BIAS_ACC, 0);
    Eigen::Vector3f gyroBias = x.block<3, 1>(IDX_BIAS_GYRO, 0);

    // State prediction

    Eigen::Vector3f correctedAngularSpeed = angularSpeed - gyroBias;
    Eigen::Vector3f correctedAcceleration = acceleration - accBias;

    Eigen::Matrix3f A;
    A.setZero();
    A <<  // Define matrix A needed for state prediction
        quat(0) * quat(0) - quat(1) * quat(1) - quat(2) * quat(2) +
            quat(3) * quat(3),
        2 * (quat(0) * quat(1) + quat(2) * quat(3)),
        2 * (quat(0) * quat(2) - quat(1) * quat(3)),
        2 * (quat(0) * quat(1) - quat(2) * quat(3)),
        -quat(0) * quat(0) + quat(1) * quat(1) - quat(2) * quat(2) +
            quat(3) * quat(3),
        2 * (quat(1) * quat(2) + quat(0) * quat(3)),
        2 * (quat(0) * quat(2) + quat(1) * quat(3)),
        2 * (quat(1) * quat(2) - quat(0) * quat(3)),
        -quat(0) * quat(0) - quat(1) * quat(1) + quat(2) * quat(2) +
            quat(3) * quat(3);

    Eigen::Matrix4f Omega;
    Omega.setZero();
    Omega <<  // Define matrix Omega needed for state state prediction
        0,
        correctedAngularSpeed(2), -correctedAngularSpeed(1),
        correctedAngularSpeed(0), -correctedAngularSpeed(2), 0,
        -correctedAngularSpeed(0), correctedAngularSpeed(1),
        correctedAngularSpeed(1), -correctedAngularSpeed(0), 0,
        correctedAngularSpeed(2), -correctedAngularSpeed(0),
        -correctedAngularSpeed(1), -correctedAngularSpeed(2), 0;

    Eigen::Vector4f predictedQuat =
        ((Eigen::Matrix4f::Identity() + 0.5f * Omega * config.T) * quat)
            .normalized();

    Eigen::Vector3f predictedVel =
        vel + config.T * (A.transpose() * correctedAcceleration + gravityNed);
    Eigen::Vector3f predictedPos      = pos + config.T * vel;
    Eigen::Vector3f predictedAccBias  = accBias;
    Eigen::Vector3f predictedGyroBias = gyroBias;
    x <<  // Assemble the predicted state
        predictedQuat,
        predictedVel, predictedPos, predictedAccBias, predictedGyroBias;

    // State covariance matrix prediction
    Eigen::Matrix3f MA;
    MA.setZero();
    MA <<  // Define matrix MA needed for matrix F computation
        0,
        -correctedAcceleration(2), correctedAcceleration(1),
        correctedAcceleration(2), 0, -correctedAcceleration(0),
        -correctedAcceleration(1), correctedAcceleration(0), 0;

    Eigen::Matrix<float, 15, 15>
        F;  // Define matrix F needed for state covariance matrix prediction
    F.setZero();  // !!!!This should be sparse ask if it is a problem
    F.block<3, 3>(0, 0)  = -Omega.block<3, 3>(0, 0);
    F.block<3, 3>(0, 12) = -Eigen::Matrix3f::Identity();
    F.block<3, 3>(3, 6)  = Eigen::Matrix3f::Zero();
    F.block<3, 3>(3, 9)  = -A;
    F.block<3, 3>(7, 3)  = Eigen::Matrix3f::Identity();
    F.block<3, 3>(3, 0)  = -A * MA;
    F                    = (F * config.T);
    F                    = F.exp();

    Eigen::Matrix<float, 15, 12>
        G;  // Define matrix G needed for state covariance matrix prediction
    G.setZero();  // !!This should be sparse ask if it is a problem
    G.block<3, 3>(0, 0) = -Eigen::Matrix3f::Identity();
    G.block<3, 3>(3, 0) = Eigen::Matrix3f::Identity();
    G.block<3, 3>(9, 9) = Eigen::Matrix3f::Identity();
    G.block<3, 3>(3, 6) = -A;
    G.block<3, 3>(3, 9) = Eigen::Matrix3f::Identity();

    P = F * P * F.transpose() +
        (F * G) * Q * (F * G).transpose();  // Covariance prediction
}

void ZVK::predict(const AccelerometerData& acceleration,
                  const GyroscopeData& angularSpeed)
{
    Eigen::Vector3f accelerationVec = {acceleration.accelerationX,
                                       acceleration.accelerationY,
                                       acceleration.accelerationZ};
    Eigen::Vector3f angularSpeedVec = {angularSpeed.angularSpeedX,
                                       angularSpeed.angularSpeedY,
                                       angularSpeed.angularSpeedZ};
    ZVK::predict(accelerationVec, angularSpeedVec);
}

void ZVK::correct(const Eigen::Vector3f& acceleration,
                  const Eigen::Vector3f& angularSpeed,
                  const Eigen::Vector3f& mag)
{
    Eigen::Vector4f quat     = x.block<4, 1>(IDX_QUAT, 0);
    Eigen::Vector3f vel      = x.block<3, 1>(IDX_VEL, 0);
    Eigen::Vector3f pos      = x.block<3, 1>(IDX_POS, 0);
    Eigen::Vector3f accBias  = x.block<3, 1>(IDX_BIAS_ACC, 0);
    Eigen::Vector3f gyroBias = x.block<3, 1>(IDX_BIAS_GYRO, 0);

    // State and covariance correction

    Eigen::Vector3f correctedAngularSpeed     = angularSpeed - gyroBias;
    Eigen::Vector3f correctedMag              = mag.normalized();
    Eigen::Vector3f correctedMagNed           = config.NED_MAG.normalized();
    Eigen::Vector3f biasCorrectedAcceleration = acceleration - accBias;

    Eigen::Matrix<float, 4, 3> OM;
    OM.setZero();
    OM <<  // Define matrix A needed for correction
        quat(3),
        -quat(2), quat(1), quat(2), quat(3), -quat(0), -quat(1), quat(0),
        quat(3), -quat(0), -quat(1), -quat(2);

    Eigen::Matrix3f A;
    A.setZero();
    A <<  // Define matrix A needed for correction
        quat(0) * quat(0) - quat(1) * quat(1) - quat(2) * quat(2) +
            quat(3) * quat(3),
        2 * (quat(0) * quat(1) + quat(2) * quat(3)),
        2 * (quat(0) * quat(2) - quat(1) * quat(3)),
        2 * (quat(0) * quat(1) - quat(2) * quat(3)),
        -quat(0) * quat(0) + quat(1) * quat(1) - quat(2) * quat(2) +
            quat(3) * quat(3),
        2 * (quat(1) * quat(2) + quat(0) * quat(3)),
        2 * (quat(0) * quat(2) + quat(1) * quat(3)),
        2 * (quat(1) * quat(2) - quat(0) * quat(3)),
        -quat(0) * quat(0) - quat(1) * quat(1) + quat(2) * quat(2) +
            quat(3) * quat(3);

    Eigen::Vector3f z = A * correctedMagNed;  // Magnetic vector in body axis
    Eigen::Vector3f estimatedAcceleration =
        A * gravityNed;  // KF estimated acceleration

    Eigen::Matrix3f M;
    M.setZero();
    M <<  // Define matrix M needed for matrix H_x initialization
        0,
        -estimatedAcceleration(2), estimatedAcceleration(1),
        estimatedAcceleration(2), 0, -estimatedAcceleration(0),
        -estimatedAcceleration(1), estimatedAcceleration(0), 0;

    Eigen::Matrix3f Z_mat;
    Z_mat.setZero();
    Z_mat <<  // Define matrix Z_mat needed for matrix H_x initialization
        0,
        -z(2), z(1), z(2), 0, -z(0), -z(1), z(0), 0;

    Eigen::Matrix<float, 12, 15>
        H_x;  // Define matrix H_x needed for correction
    H_x.setZero();
    H_x.block<3, 3>(0, 0)  = Eigen::Matrix3f::Zero();
    H_x.block<3, 3>(0, 3)  = -Eigen::Matrix3f::Identity();
    H_x.block<3, 3>(0, 6)  = Eigen::Matrix3f::Zero();
    H_x.block<3, 3>(0, 9)  = Eigen::Matrix3f::Zero();
    H_x.block<3, 3>(0, 12) = Eigen::Matrix3f::Zero();
    H_x.block<3, 3>(3, 0)  = Eigen::Matrix3f::Zero();
    H_x.block<3, 3>(3, 3)  = Eigen::Matrix3f::Zero();
    H_x.block<3, 3>(3, 6)  = Eigen::Matrix3f::Zero();
    H_x.block<3, 3>(3, 9)  = Eigen::Matrix3f::Zero();
    H_x.block<3, 3>(3, 12) = -Eigen::Matrix3f::Identity();
    H_x.block<3, 3>(6, 0)  = M;
    H_x.block<3, 3>(6, 3)  = Eigen::Matrix3f::Zero();
    H_x.block<3, 3>(6, 6)  = Eigen::Matrix3f::Zero();
    H_x.block<3, 3>(6, 9)  = Eigen::Matrix3f::Identity();
    H_x.block<3, 3>(6, 12) = Eigen::Matrix3f::Zero();
    H_x.block<3, 3>(9, 0)  = Z_mat;
    H_x.block<3, 3>(9, 3)  = Eigen::Matrix3f::Zero();
    H_x.block<3, 3>(9, 6)  = Eigen::Matrix3f::Zero();
    H_x.block<3, 3>(9, 9)  = Eigen::Matrix3f::Zero();
    H_x.block<3, 3>(9, 12) = Eigen::Matrix3f::Zero();

    Eigen::Matrix<float, 12, 12> S =
        H_x * P * H_x.transpose() +
        R;  // Define matrix S needed for gain computation
    Eigen::Matrix<float, 15, 12> K =
        P * H_x.transpose() * S.inverse();  // Define the gain K

    // Error computation

    Eigen::Matrix<float, 12, 1> errorMinuend;
    errorMinuend.block<6, 1>(0, 0) = Eigen::Matrix<float, 6, 1>::Zero();
    errorMinuend.block<3, 1>(6, 0) = biasCorrectedAcceleration;
    errorMinuend.block<3, 1>(9, 0) = correctedMag;

    Eigen::Matrix<float, 12, 1> errorSubtrahend;
    errorSubtrahend.block<3, 1>(0, 0) = vel;
    errorSubtrahend.block<3, 1>(3, 0) = correctedAngularSpeed;
    errorSubtrahend.block<3, 1>(6, 0) = estimatedAcceleration;
    errorSubtrahend.block<3, 1>(9, 0) = z;

    Eigen::Matrix<float, 12, 1> error = errorMinuend - errorSubtrahend;

    // Update computation
    Eigen::Matrix<float, 15, 1> update = K * error;

    // State correction
    x.block<12, 1>(4, 0) = x.block<12, 1>(4, 0) + update.block<12, 1>(3, 0);
    x.block<4, 1>(0, 0) =
        (quat + 0.5 * OM * update.block<3, 1>(0, 0)).normalized();

    // Covariance correction
    P = (Eigen::Matrix<float, 15, 15>::Identity() - K * H_x) * P;
}

void ZVK::correct(const AccelerometerData& acceleration,
                  const GyroscopeData& angularSpeed,
                  const MagnetometerData& mag)
{
    Eigen::Vector3f accelerationVec = {acceleration.accelerationX,
                                       acceleration.accelerationY,
                                       acceleration.accelerationZ};
    Eigen::Vector3f angularSpeedVec = {angularSpeed.angularSpeedX,
                                       angularSpeed.angularSpeedY,
                                       angularSpeed.angularSpeedZ};
    Eigen::Vector3f magVec          = {mag.magneticFieldX, mag.magneticFieldY,
                                       mag.magneticFieldZ};
    ZVK::correct(accelerationVec, angularSpeedVec, magVec);
}

ZVKState ZVK::getState() const
{
    return ZVKState(TimestampTimer::getTimestamp(), this->x);
}

void ZVK::setState(const ZVKState& state) { this->x = state.getX(); }

Eigen::Matrix<float, 16, 1> ZVK::getX() const { return this->x; }

void ZVK::setX(const Eigen::Matrix<float, 16, 1>& x) { this->x = x; }

}  // namespace Boardcore
