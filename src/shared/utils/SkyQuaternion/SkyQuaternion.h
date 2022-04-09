/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Marco Cella
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
 * @brief Functions for managing quaternions.
 *
 * Convention used: x, y, z, w (scalar element as last element)
 */
namespace SkyQuaternion
{

/**
 * @brief Transform a vector of euler angles (ZYX) to quaternion.
 *
 * @param euler The vector of euler angles to be transformed [deg]
 * @return Transformed quaternion [x, y, z, w]
 */
Eigen::Vector4f eul2quat(Eigen::Vector3f euler);

/**
 * @brief Transform a quaternion to a vector of euler angles (ZYX).
 *
 * @param quat The quaternion to be transformed [x, y, z, w]
 * @return Transformed vector of euler angles [deg]
 */
Eigen::Vector3f quat2eul(Eigen::Vector4f quat);

/**
 * @brief Transform a rotation matrix to a quaternion.
 *
 * @param rtm The rotation matrix to be transformed (3x3)
 * @return Transformed quaternion [x, y, z, w]
 */
Eigen::Vector4f rotationMatrix2quat(Eigen::Matrix3f rtm);

/**
 * @brief Normalize a quaternion.
 *
 * @param quat The quaternion to be normalized [x, y, z, w]
 * @return True if the operation succeeded or not
 */
bool quatNormalize(Eigen::Vector4f& quat);

/**
 * @brief Compute the product of two quaternions.
 *
 * @param q1 First factor [x, y, z, w]
 * @param q2 Second factor [x, y, z, w]
 * @return The resulting quaternions product [x, y, z, w]
 */
Eigen::Vector4f quatProd(const Eigen::Vector4f q1, const Eigen::Vector4f q2);

};  // namespace SkyQuaternion

}  // namespace Boardcore
