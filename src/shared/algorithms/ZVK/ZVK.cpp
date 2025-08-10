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
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <unsupported/Eigen/MatrixFunctions>


using namespace Boardcore;


namespace Boardcore
{

ZVK::ZVK(const ZVKConfig& config): config(config)
{
    // Covariance matrices setup

    // State covariance matrix setup
    {
    Eigen::Matrix<float, 12, 1> diagElements;
    // Set elements in the vector 
    diagElements <<
        Eigen::Vector3f::Ones() * smallAngleError * smallAngleError,
        Eigen::Vector3f::Ones() * config.VELOCITY_UNCERTAINTY,
        Eigen::Vector3f::Ones() * config.POSITION_UNCERTAINTY,
        Eigen::Vector3f::Ones() * config.BIAS_ACC * config.BIAS_ACC,
        Eigen::Vector3f::Ones() * config.BIAS_GYRO * config.BIAS_GYRO;
    P = diagElements.asDiagonal();
    }

    // Measurement noise covariance matrix setup
    {
    R.setZero();
    // Set diagonal 3x3 blocks
    R.block<3,3>(0, 0)  = rConst * Eigen::Matrix3f::Identity();               // first block
    R.block<3,3>(3, 3)  = rConst * Eigen::Matrix3f::Identity();               // second block
    R.block<3,3>(6, 6)  = config.SIGMA_ACC * config.SIGMA_ACC * Eigen::Matrix3f::Identity(); // third block
    R.block<3,3>(9, 9)  = config.SIGMA_MAG * config.SIGMA_MAG * Eigen::Matrix3f::Identity(); // fourth block
    }

    // Process noise covariance matrix setup
    {
    Eigen::Matrix<float, 12, 1> diagElements;
    // Set elements in the vector
    diagElements <<
        Eigen::Vector3f::Ones() * config.SIGMA_GYRO * config.SIGMA_GYRO,
        Eigen::Vector3f::Ones() * config.SIGMA_GYRO_BIAS * config.SIGMA_GYRO_BIAS,
        Eigen::Vector3f::Ones() * config.SIGMA_ACC * config.SIGMA_ACC,
        Eigen::Vector3f::Ones() * config.SIGMA_GYRO_BIAS  * config.SIGMA_GYRO_BIAS;
    Q = diagElements.asDiagonal();
    }
}

void ZVK::predict(const Eigen::Vector3f& acceleration, const Eigen::Vector3f& angularSpeed)
{ 
    Eigen::Vector4f quat = x.block<4,1>(IDX_QUAT, 0);
    Eigen::Vector3f vel = x.block<3,1>(IDX_VEL, 0);
    Eigen::Vector3f pos = x.block<3,1>(IDX_POS, 0);
    Eigen::Vector3f accBias = x.block<3,1>(IDX_BIAS_ACC, 0);
    Eigen::Vector3f gyroBias = x.block<3,1>(IDX_BIAS_GYRO, 0);

    // State prediction

    Eigen::Vector3f correctedAngularSpeed = angularSpeed - gyroBias;
    Eigen::Vector3f correctedAcceleration = acceleration - accBias;

    Eigen::Matrix3f A; 
    A <<  // Define matrix A needed for state prediction
    quat(0)*quat(0) - quat(1)*quat(1) - quat(2)*quat(2) + quat(3)*quat(3),  2*(quat(0)*quat(1) + quat(2)*quat(3)), 2*(quat(0)*quat(2) - quat(1)*quat(3)),
    2*(quat(0)*quat(1) - quat(2)*quat(3)), -quat(0)*quat(0) + quat(1)*quat(1) - quat(2)*quat(2) + quat(3)*quat(3), 2*(quat(1)*quat(2) + quat(0)*quat(3)),
    2*(quat(0)*quat(2) + quat(1)*quat(3)), 2*(quat(1)*quat(2) - quat(0)*quat(3)), -quat(0)*quat(0) - quat(1)*quat(1) + quat(2)*quat(2) + quat(3)*quat(3);
    
    Eigen::Matrix4f Omega;
    Omega << // Define matrix Omega needed for state state prediction 
    0, correctedAngularSpeed(2), -correctedAngularSpeed(1), correctedAngularSpeed(0),
    -correctedAngularSpeed(2), 0, -correctedAngularSpeed(0), correctedAngularSpeed(1),
    correctedAngularSpeed(1), -correctedAngularSpeed(0), 0, correctedAngularSpeed(2),
    -correctedAngularSpeed(0), -correctedAngularSpeed(1), -correctedAngularSpeed(2), 0;

    Eigen::Vector4f predictedQuat = ((Eigen::Matrix4d::Identity() + 0.5 * Omega * config.T) * quat).normalized();
    Eigen::Vector3f predictedVel = vel + config.T * (A.transpose() * correctedAcceleration + gravityNed);
    Eigen::Vector3f predictedPos = pos + config.T * vel; 
    Eigen::Vector3f predictedAccBias = accBias;
    Eigen::Vector3f predictedGyroBias = gyroBias;
    x << // Assemble the predicted state
    predictedQuat, 
    predictedVel,
    predictedPos,
    predictedAccBias,
    predictedGyroBias;

    // State covariance matrix prediction 
    Eigen::Matrix3f MA; //
    MA << //Define matrix MA needed for matrix F computation
    0, -correctedAcceleration(2), correctedAcceleration(1),
    correctedAcceleration(2), 0, -correctedAcceleration(0),
    -correctedAcceleration(1), correctedAcceleration(0), 0;

    Eigen::Matrix<float, 15, 15> F; //Define matrix F needed for state covariance matrix prediction !!!!This should be sparse ask if it is a problem
    F.block<3,3>(0,0) = -Omega.block<3,3>(0,0);
    F.block<3,3>(0,12) = -Eigen::Matrix3f::Identity();
    F.block<3,3>(3,6) = Eigen::Matrix3f::Zero();
    F.block<3,3>(3,9) = -A;
    F.block<3,3>(7,3) = Eigen::Matrix3f::Identity();
    F.block<3,3>(3,0) = -A * MA;
    F = ( F * config.T );
    F = F.exp();

    Eigen::Matrix<float, 15, 15> G; //Define matrix F needed for state covariance matrix prediction !!This should be sparse ask if it is a problem
    G.block<3,3>(0,0) = -Eigen::Matrix3f::Identity();
    G.block<3,3>(3,0) = Eigen::Matrix3f::Identity();
    G.block<3,3>(9,9) = Eigen::Matrix3f::Identity();
    G.block<3,3>(3,6) = -A;
    G.block<3,3>(3,9) = Eigen::Matrix3f::Identity();
    
    P = F * P * F.transpose() + (F * G) * Q * (F * G).transpose();     // Covariance prediction
}

void ZVK::predict(const AccelerometerData& acceleration, const GyroscopeData& angularSpeed)
{
    Eigen::Vector3f accelerationVec = {acceleration.accelerationX, acceleration.accelerationY, acceleration.accelerationZ};
    Eigen::Vector3f angularSpeedVec = {angularSpeed.angularSpeedX, angularSpeed.angularSpeedY, angularSpeed.angularSpeedZ};
    ZVK::predict(accelerationVec, angularSpeedVec);
}



} //namespace Boardcore