/* Quaternion
 *
 * Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Marco Cella
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
using namespace Eigen;

/**
 * @brief Class for managing quaternions.
 */
class SkyQuaternion
{

public:
    SkyQuaternion();

    /**
     * @brief Transform a vector of euler angles ZYX to quaternion.
     *
     * @param degeul the vector of euler angles to be transformed [deg]
     *
     * @return transformed quaternion
     */
    Vector4f eul2quat(Vector3f degeul);

    /**
     * @brief Transform a quaternion to a vector of euler angles.
     *
     * @param quat the quaternion to be transformed
     *
     * @return transformed vector of euler angles [deg]
     */
    Vector3f quat2eul(Vector4f quat);

    /**
     * @brief Transform a rotation matrix to a quaternion.
     *
     * @param R the rotation matrix to be transformed (3x3)
     *
     * @return transformed quaternion
     */
    Vector4f rotm2quat(Matrix3f R);

    /**
     * @brief Normalize a quaternion.
     *
     * @param quat the quaternion to be normalized
     *
     * @return boolean indicating if the operation succeded or not
     */
    bool quatnormalize(Vector4f& quat);

    /**
     * @brief Compute the product of two quaternions.
     *
     * @param q1 the first factor
     * @param q2 the second factor
     *
     * @return the resulting quaternions product
     */
    Vector4f quatProd(const Vector4f q1, const Vector4f q2);
};
