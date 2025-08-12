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

#include "ZVK.h"

#include <drivers/timer/TimestampTimer.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "ZVKState.h"

using namespace Boardcore;
namespace Boardcore
{

ZVK::ZVK(const ZVKConfig& config)
    : config(config),
      onRampQuaternion(SkyQuaternion::eul2quat(config.ON_RAMP_EULERO_ANGLES))
{
    // Q initialization
    {
        // clang-format-off
        Eigen::Matrix<float, 24, 1> qDiag;
        qDiag << Eigen::Vector3f::Constant((1e-6f) * (1e-6f)),
            Eigen::Vector3f::Constant((1e-6f) * (1e-6f)),
            Eigen::Vector3f::Constant(config.SIGMA_BIAS_ACC *
                                      config.SIGMA_BIAS_ACC),
            Eigen::Vector3f::Constant(config.SIGMA_BIAS_ACC *
                                      config.SIGMA_BIAS_ACC),
            Eigen::Vector3f::Constant((1e-6f) * (1e-6f)),
            Eigen::Vector3f::Constant((1e-6f) * (1e-6f)),
            Eigen::Vector3f::Constant(config.SIGMA_BIAS_GYRO *
                                      config.SIGMA_BIAS_GYRO),
            Eigen::Vector3f::Constant(config.SIGMA_BIAS_GYRO *
                                      config.SIGMA_BIAS_GYRO);

        Q = qDiag.asDiagonal();
        Q.makeCompressed();
        // clang-format-on
    }

    // P initialization
    {
        // clang-format-off
        P = std::make_unique<Eigen::Matrix<float, 24, 24>>();
        Eigen::Matrix<float, 24, 1> p0Diag;
        p0Diag << Eigen::Vector3f::Constant((1e-5f) * (1e-5f)),
            Eigen::Vector3f::Constant((1e-5f) * (1e-5f)),
            Eigen::Vector3f::Constant((1e-1f) * (1e-1f)),
            Eigen::Vector3f::Constant((1e-1f) * (1e-1f)),
            Eigen::Vector3f::Constant((1e-5f) * (1e-5f)),
            Eigen::Vector3f::Constant((1e-5f) * (1e-5f)),
            Eigen::Vector3f::Constant((1e-3f) * (1e-3f)),
            Eigen::Vector3f::Constant((1e-3f) * (1e-3f));
        P->block<24, 24>(0, 0) = p0Diag.asDiagonal();
        // clang-format-on
    }

    // R_ZERO_VEL initialization
    {
        // clang-format-off
        Eigen::Matrix<float, 6, 1> rZeroVelDiag;
        rZeroVelDiag << Eigen::Vector3f::Constant((1e-6f) * (1e-6f)),
            Eigen::Vector3f::Constant((1e-6f) * (1e-6f));
        R_ZERO_VEL = rZeroVelDiag.asDiagonal();
        R_ZERO_VEL.makeCompressed();
        // clang-format-on
    }

    // R_ACC initialization
    {
        // clang-format-off
        Eigen::Matrix<float, 3, 1> rAccDiag;
        rAccDiag << Eigen::Vector3f::Constant(config.SIGMA_ACC *
                                              config.SIGMA_ACC);
        R_ACC = rAccDiag.asDiagonal();
        R_ACC.makeCompressed();
        // clang-format-on
    }

    // R_GYRO initialization
    {
        // clang-format-off
        Eigen::Matrix<float, 3, 1> rGyroDiag;
        rGyroDiag << Eigen::Vector3f::Constant(config.SIGMA_GYRO *
                                               config.SIGMA_GYRO);
        R_GYRO = rGyroDiag.asDiagonal();
        R_GYRO.makeCompressed();
        // clang-format-on
    }

    // H_ZERO_VEL initialization
    {
        // clang-format-off
        Eigen::Matrix<float, 6, 6> tempHZeroVel =
            Eigen::Matrix<float, 6, 6>::Zero();
        tempHZeroVel.block<3, 3>(0, IDX_VEL)     = Eigen::Matrix3f::Identity();
        tempHZeroVel.block<3, 3>(3, IDX_EUL_ANG) = Eigen::Matrix3f::Identity();
        H_ZERO_VEL                               = tempHZeroVel.sparseView();
        // clang-format-on
    }

    // H_ACC_GYRO initialization
    {
        // clang-format-off
        Eigen::Matrix<float, 3, 6> tempHAccGyro =
            Eigen::Matrix<float, 3, 6>::Zero();
        tempHAccGyro.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
        tempHAccGyro.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity();
        H_ACC_GYRO                     = tempHAccGyro.sparseView();
        //  clang-format-on
    }

    // A_ROT initialization
    {
        // clang-format-off
        float q1 = onRampQuaternion(0);
        float q2 = onRampQuaternion(1);
        float q3 = onRampQuaternion(2);
        float q4 = onRampQuaternion(3);

        A_ROT << q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4, 2 * (q1 * q2 + q3 * q4),
            2 * (q1 * q3 - q2 * q4), 2 * (q1 * q2 - q3 * q4),
            -q1 * q1 + q2 * q2 - q3 * q3 + q4 * q4, 2 * (q2 * q3 + q1 * q4),
            2 * (q1 * q3 + q2 * q4),
            2 * (q2 * q3 - q1 * q4),  // cppcheck-suppress constStatement
            -q1 * q1 - q2 * q2 + q3 * q3 + q4 * q4;
        // clang-format-on
    }

    // F initialization
    {
        // clang-format-off
        F.resize(24, 24);

        // Set 3x3 diagonal matrix in (0, 3)
        F.coeffRef(0, 3) = config.T;
        F.coeffRef(1, 4) = config.T;
        F.coeffRef(2, 5) = config.T;

        // Set 3x3 diagonal matrix in (12, 15)
        F.coeffRef(12, 15) = config.T;
        F.coeffRef(13, 16) = config.T;
        F.coeffRef(14, 17) = config.T;
        // clang-format-on
    }
}

void ZVK::predict()
{
    // Extract state
    const Eigen::Vector3f vel       = x.block<3, 1>(IDX_VEL, 0);
    const Eigen::Vector3f acc       = x.block<3, 1>(IDX_ACC, 0);
    const Eigen::Vector3f biasAcc0  = x.block<3, 1>(IDX_BIAS_ACC_0, 0);
    const Eigen::Vector3f biasAcc1  = x.block<3, 1>(IDX_BIAS_ACC_1, 0);
    const Eigen::Vector3f eulAng    = x.block<3, 1>(IDX_EUL_ANG, 0);
    const Eigen::Vector3f angVel    = x.block<3, 1>(IDX_VEL_ANG, 0);
    const Eigen::Vector3f biasGyro0 = x.block<3, 1>(IDX_BIAS_GYRO_0, 0);
    const Eigen::Vector3f biasGyro1 = x.block<3, 1>(IDX_BIAS_GYRO_1, 0);

    // Compute state prediction
    Eigen::Vector3f velPred       = vel + config.T * acc;
    Eigen::Vector3f accPred       = acc;
    Eigen::Vector3f biasAcc0Pred  = biasAcc0;
    Eigen::Vector3f biasAcc1Pred  = biasAcc1;
    Eigen::Vector3f eulAngPred    = eulAng + config.T * angVel;
    Eigen::Vector3f angVelPred    = angVel;
    Eigen::Vector3f biasGyro0Pred = biasGyro0;
    Eigen::Vector3f biasGyro1Pred = biasGyro1;

    // Update state
    x.block<3, 1>(IDX_VEL, 0)         = velPred;
    x.block<3, 1>(IDX_ACC, 0)         = accPred;
    x.block<3, 1>(IDX_BIAS_ACC_0, 0)  = biasAcc0Pred;
    x.block<3, 1>(IDX_BIAS_ACC_1, 0)  = biasAcc1Pred;
    x.block<3, 1>(IDX_EUL_ANG, 0)     = eulAngPred;
    x.block<3, 1>(IDX_VEL_ANG, 0)     = angVelPred;
    x.block<3, 1>(IDX_BIAS_GYRO_0, 0) = biasGyro0Pred;
    x.block<3, 1>(IDX_BIAS_GYRO_1, 0) = biasGyro1Pred;

    // update P
    *P = F * (*P) * F.transpose() + Q;
}

void ZVK::correctZeroVel()
{
    const Eigen::Vector3f vel          = x.block<3, 1>(IDX_VEL, 0);
    const Eigen::Vector3f eulAng       = x.block<3, 1>(IDX_EUL_ANG, 0);
    const Eigen::Vector3f onRampVel    = Eigen::Vector3f::Zero();
    const Eigen::Vector3f onRampEulAng = config.ON_RAMP_EULERO_ANGLES;

    // Extract subP used in kalman gain computation
    Eigen::Matrix<float, 6, 6> subP;
    subP.block<3, 3>(0, 0) = P->block<3, 3>(0, 0);
    subP.block<3, 3>(0, 3) = P->block<3, 3>(0, 12);
    subP.block<3, 3>(3, 0) = P->block<3, 3>(12, 0);
    subP.block<3, 3>(3, 3) = P->block<3, 3>(12, 12);

    // Compute matrix S needed for kalman gain computation
    Eigen::Matrix<float, 6, 6> S =
        H_ZERO_VEL * subP * H_ZERO_VEL.transpose() + R_ZERO_VEL;

    // If S is not invertible, don't do the correction and return
    if (S.determinant() < 1e-3)
        return;

    // Compute kalman gain
    Eigen::Matrix<float, 6, 6> K =
        (subP * H_ZERO_VEL.transpose() * S.inverse());

    // Compute error
    Eigen::Matrix<float, 6, 1> error = Eigen::Matrix<float, 6, 1>::Zero(6);
    error.block<3, 1>(0, 0)          = onRampVel - vel;
    error.block<3, 1>(3, 0)          = onRampEulAng - eulAng;

    // Compute update
    Eigen::Matrix<float, 6, 1> update = K * error;

    // Update velocity and euler angles in state
    x.block<3, 1>(IDX_VEL, 0)     = update.block<3, 1>(0, 0);
    x.block<3, 1>(IDX_EUL_ANG, 0) = update.block<3, 1>(3, 0);

    // Update P
    P->block<3, 3>(0, 0)   = subP.block<3, 3>(0, 0);
    P->block<3, 3>(0, 12)  = subP.block<3, 3>(0, 3);
    P->block<3, 3>(12, 0)  = subP.block<3, 3>(3, 0);
    P->block<3, 3>(12, 12) = subP.block<3, 3>(3, 3);
}

void ZVK::correctAcc0(const Eigen::Vector3f& accMeas)
{
    const Eigen::Vector3f acc      = x.block<3, 1>(IDX_ACC, 0);
    const Eigen::Vector3f biasAcc0 = x.block<3, 1>(IDX_BIAS_ACC_0, 0);

    // Extract subP used in kalman gain computation
    Eigen::Matrix<float, 6, 6> subP = P->block<6, 6>(3, 3);

    // Compute matrix S needed for kalman gain computation
    Eigen::Matrix<float, 3, 3> S =
        H_ACC_GYRO * subP * H_ACC_GYRO.transpose() + R_ACC;

    // If S is not invertible, don't do the correction and return
    if (S.determinant() < 1e-3)
        return;

    // Compute kalman gain
    Eigen::Matrix<float, 6, 3> K =
        (subP * H_ACC_GYRO.transpose() * S.inverse());

    // Accelerometer correction
    Eigen::Vector3f correctedAcc =
        accMeas - A_ROT * gravityNed;  // In NED frame without gravity
    Eigen::Vector3f biasCorrectedAcc = acc + biasAcc0;

    // Compute error
    Eigen::Matrix<float, 3, 1> error = correctedAcc - biasCorrectedAcc;

    // Compute update
    Eigen::Matrix<float, 6, 1> update = K * error;

    // Update acceleration and bias in state
    x.block<3, 1>(IDX_ACC, 0)        = update.block<3, 1>(0, 0);
    x.block<3, 1>(IDX_BIAS_ACC_0, 0) = update.block<3, 1>(2, 0);

    // Update P
    P->block<6, 6>(3, 3) = subP;
}

void ZVK::correctAcc1(const Eigen::Vector3f& accMeas)
{
    const Eigen::Vector3f acc      = x.block<3, 1>(IDX_ACC, 0);
    const Eigen::Vector3f biasAcc1 = x.block<3, 1>(IDX_BIAS_ACC_1, 0);

    // Extract subP used in kalman gain computation
    Eigen::Matrix<float, 6, 6> subP;
    subP.block<3, 3>(0, 0) = P->block<3, 3>(3, 3);
    subP.block<3, 3>(0, 3) = P->block<3, 3>(3, 9);
    subP.block<3, 3>(3, 0) = P->block<3, 3>(9, 3);
    subP.block<3, 3>(3, 3) = P->block<3, 3>(9, 9);

    // Compute matrix S needed for kalman gain computation
    Eigen::Matrix<float, 3, 3> S =
        H_ACC_GYRO * subP * H_ACC_GYRO.transpose() + R_ACC;

    // If S is not invertible, don't do the correction and return
    if (S.determinant() < 1e-3)
        return;

    // Compute kalman gain
    Eigen::Matrix<float, 6, 3> K =
        (subP * H_ACC_GYRO.transpose() * S.inverse());

    // Accelerometer correction
    Eigen::Vector3f correctedAcc =
        accMeas - A_ROT * gravityNed;  // In NED frame without gravity
    Eigen::Vector3f biasCorrectedAcc = acc + biasAcc1;

    // Compute error
    Eigen::Matrix<float, 3, 1> error = correctedAcc - biasCorrectedAcc;

    // Compute update
    Eigen::Matrix<float, 6, 1> update = K * error;

    // Update acceleration and bias in state
    x.block<3, 1>(IDX_ACC, 0)        = update.block<3, 1>(0, 0);
    x.block<3, 1>(IDX_BIAS_ACC_1, 0) = update.block<3, 1>(2, 0);

    // Update P
    P->block<3, 3>(3, 3) = subP.block<3, 3>(0, 0);
    P->block<3, 3>(3, 9) = subP.block<3, 3>(0, 3);
    P->block<3, 3>(9, 3) = subP.block<3, 3>(3, 0);
    P->block<3, 3>(9, 9) = subP.block<3, 3>(3, 3);
}

void ZVK::correctGyro0(const Eigen::Vector3f& angVelMeas)
{
    const Eigen::Vector3f angVel    = x.block<3, 1>(IDX_VEL_ANG, 0);
    const Eigen::Vector3f biasGyro0 = x.block<3, 1>(IDX_BIAS_GYRO_0, 0);

    // Extract subP used in kalman gain computation
    Eigen::Matrix<float, 6, 6> subP = P->block<6, 6>(15, 15);

    // Compute matrix S needed for kalman gain computation
    Eigen::Matrix<float, 3, 3> S =
        H_ACC_GYRO * subP * H_ACC_GYRO.transpose() + R_GYRO;

    // If S is not invertible, don't do the correction and return
    if (S.determinant() < 1e-3)
        return;

    // Compute kalman gain
    Eigen::Matrix<float, 6, 3> K =
        (subP * H_ACC_GYRO.transpose() * S.inverse());

    // Gyroscpe correction
    Eigen::Vector3f estimatedAngVel = angVel + biasGyro0;

    // Compute error
    Eigen::Matrix<float, 3, 1> error = angVelMeas - estimatedAngVel;

    // Compute update
    Eigen::Matrix<float, 6, 1> update = K * error;

    // Update acceleration and bias in state
    x.block<3, 1>(IDX_VEL_ANG, 0)     = update.block<3, 1>(0, 0);
    x.block<3, 1>(IDX_BIAS_GYRO_0, 0) = update.block<3, 1>(2, 0);

    // Update P
    P->block<6, 6>(15, 15) = subP;
}

void ZVK::correctGyro1(const Eigen::Vector3f& angVelMeas)
{
    const Eigen::Vector3f angVel    = x.block<3, 1>(IDX_VEL_ANG, 0);
    const Eigen::Vector3f biasGyro1 = x.block<3, 1>(IDX_BIAS_GYRO_1, 0);

    // Extract subP used in kalman gain computation
    Eigen::Matrix<float, 6, 6> subP;
    subP.block<3, 3>(0, 0) = P->block<3, 3>(15, 15);
    subP.block<3, 3>(0, 3) = P->block<3, 3>(15, 21);
    subP.block<3, 3>(3, 0) = P->block<3, 3>(21, 15);
    subP.block<3, 3>(3, 3) = P->block<3, 3>(21, 21);

    // Compute matrix S needed for kalman gain computation
    Eigen::Matrix<float, 3, 3> S =
        H_ACC_GYRO * subP * H_ACC_GYRO.transpose() + R_GYRO;

    // If S is not invertible, don't do the correction and return
    if (S.determinant() < 1e-3)
        return;

    // Compute kalman gain
    Eigen::Matrix<float, 6, 3> K =
        (subP * H_ACC_GYRO.transpose() * S.inverse());

    // Gyroscpe correction
    Eigen::Vector3f estimatedAngVel = angVel + biasGyro1;

    // Compute error
    Eigen::Matrix<float, 3, 1> error = angVelMeas - estimatedAngVel;

    // Compute update
    Eigen::Matrix<float, 6, 1> update = K * error;

    // Update acceleration and bias in state
    x.block<3, 1>(IDX_VEL_ANG, 0)     = update.block<3, 1>(0, 0);
    x.block<3, 1>(IDX_BIAS_GYRO_1, 0) = update.block<3, 1>(2, 0);

    // Update P
    P->block<3, 3>(15, 15) = subP.block<3, 3>(0, 0);
    P->block<3, 3>(15, 21) = subP.block<3, 3>(0, 3);
    P->block<3, 3>(21, 15) = subP.block<3, 3>(3, 0);
    P->block<3, 3>(21, 21) = subP.block<3, 3>(3, 3);
}

void ZVK::correctAcc0(const AccelerometerData& acceleration)
{
    correctAcc0(Eigen::Vector3f{acceleration.accelerationX,
                                acceleration.accelerationY,
                                acceleration.accelerationZ});
}

void ZVK::correctAcc1(const AccelerometerData& acceleration)
{
    correctAcc1(Eigen::Vector3f{acceleration.accelerationX,
                                acceleration.accelerationY,
                                acceleration.accelerationZ});
}

void ZVK::correctGyro0(const GyroscopeData& angularVel)
{
    correctGyro0(Eigen::Vector3f{angularVel.angularSpeedX,
                                 angularVel.angularSpeedY,
                                 angularVel.angularSpeedZ});
}

void ZVK::correctGyro1(const GyroscopeData& angularVel)
{
    correctGyro1(Eigen::Vector3f{angularVel.angularSpeedX,
                                 angularVel.angularSpeedY,
                                 angularVel.angularSpeedZ});
}

ZVKState ZVK::getState() const
{
    return ZVKState(TimestampTimer::getTimestamp(), x);
}

void ZVK::setState(const ZVKState& state) { this->x = state.getX(); }

Eigen::Matrix<float, 24, 1> ZVK::getX() const { return x; }

void ZVK::setX(const Eigen::Matrix<float, 24, 1>& x) { this->x = x; }

}  // namespace Boardcore
