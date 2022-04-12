/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Marco Cella, Alberto Nidasio
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

#include "SkyQuaternion.h"

#include <utils/Constants.h>

#include "iostream"

using namespace std;
using namespace Eigen;

namespace Boardcore
{

namespace SkyQuaternion
{

Vector4f eul2quat(Vector3f euler)
{
    // Euler angles are ZYX
    float yaw   = euler(0) * Constants::DEGREES_TO_RADIANS;
    float pitch = euler(1) * Constants::DEGREES_TO_RADIANS;
    float roll  = euler(2) * Constants::DEGREES_TO_RADIANS;

    float cyaw   = cosf(yaw * 0.5F);
    float syaw   = sinf(yaw * 0.5F);
    float cpitch = cosf(pitch * 0.5F);
    float spitch = sinf(pitch * 0.5F);
    float croll  = cosf(roll * 0.5F);
    float sroll  = sinf(roll * 0.5F);

    float qx = cyaw * cpitch * sroll - syaw * spitch * croll;
    float qy = cyaw * spitch * croll + syaw * cpitch * sroll;
    float qz = syaw * cpitch * croll - cyaw * spitch * sroll;
    float qw = cyaw * cpitch * croll + syaw * spitch * sroll;

    return Vector4f(qx, qy, qz, qw);
}

Vector3f quat2eul(Vector4f quat)
{
    // Euler angles are ZYX
    float qx = quat(0);
    float qy = quat(1);
    float qz = quat(2);
    float qw = quat(3);

    float yaw = atan2f(2.0F * (qx * qy + qw * qz),
                       powf(qw, 2) + powf(qx, 2) - powf(qy, 2) - powf(qz, 2));

    float a = -2.0F * (qx * qz - qw * qy);
    if (a > 1.0F)
        a = 1.0F;
    else if (a < -1.0F)
        a = -1.0F;

    float pitch = asinf(a);

    float roll = atan2f(2.0F * (qy * qz + qw * qx),
                        powf(qw, 2) - powf(qx, 2) - powf(qy, 2) + powf(qz, 2));

    return Vector3f(roll, pitch, yaw) * Constants::RADIANS_TO_DEGREES;
}

Vector4f rotationMatrix2quat(Matrix3f rtm)
{
    float r11 = rtm(0, 0);
    float r12 = rtm(0, 1);
    float r13 = rtm(0, 2);
    float r21 = rtm(1, 0);
    float r22 = rtm(1, 1);
    float r23 = rtm(1, 2);
    float r31 = rtm(2, 0);
    float r32 = rtm(2, 1);
    float r33 = rtm(2, 2);

    float qx;
    float qy;
    float qz;
    float qw;

    float tr = r11 + r22 + r33;

    if (tr > 0)
    {
        float sqtrp1 = sqrt(1 + tr);

        qx = (r23 - r32) / (2.0 * sqtrp1);
        qy = (r31 - r13) / (2.0 * sqtrp1);
        qz = (r12 - r21) / (2.0 * sqtrp1);
        qw = 0.5 * sqtrp1;
    }
    else
    {
        if ((r22 > r11) && (r22 > r33))
        {
            float sqdip1 = sqrt(r22 - r11 - r33 + 1.0);

            qy = 0.5 * sqdip1;

            if (sqdip1 != 0)
                sqdip1 = 0.5 / sqdip1;

            qx = (r12 + r21) * sqdip1;
            qz = (r23 + r32) * sqdip1;
            qw = (r31 - r13) * sqdip1;
        }
        else if (r33 > r11)
        {
            float sqdip1 = sqrt(r33 - r11 - r22 + 1.0);

            qz = 0.5 * sqdip1;

            if (sqdip1 != 0)
                sqdip1 = 0.5 / sqdip1;

            qx = (r31 + r13) * sqdip1;
            qy = (r23 + r32) * sqdip1;
            qw = (r12 - r21) * sqdip1;
        }
        else
        {
            float sqdip1 = sqrt(r11 - r22 - r33 + 1.0);

            qx = 0.5 * sqdip1;

            if (sqdip1 != 0)
                sqdip1 = 0.5 / sqdip1;

            qy = (r12 + r21) * sqdip1;
            qz = (r31 + r13) * sqdip1;
            qw = (r23 - r32) * sqdip1;
        }
    }

    return Vector4f(qx, qy, qz, qw);
}

bool quatNormalize(Vector4f& quat)
{
    float den = sqrt(powf(quat(0), 2) + powf(quat(1), 2) + powf(quat(2), 2) +
                     powf(quat(3), 2));
    if (den < 1e-8)
        return false;

    quat = quat / den;

    return true;
}

Vector4f quatProd(const Vector4f q1, const Vector4f q2)
{
    float q1x = q1(0);
    float q1y = q1(1);
    float q1z = q1(2);
    Vector3f qv1(q1x, q1y, q1z);
    float q1w = q1(3);

    float q2x = q2(0);
    float q2y = q2(1);
    float q2z = q2(2);
    Vector3f qv2(q2x, q2y, q2z);
    float q2w = q2(3);

    Vector4f quater;
    // cppcheck-suppress constStatement
    quater << q1w * qv2 + q2w * qv1 - qv1.cross(qv2), q1w * q2w - qv1.dot(qv2);
    float quaternionNorm = sqrt(quater(0) * quater(0) + quater(1) * quater(1) +
                                quater(2) * quater(2) + quater(3) * quater(3));
    quater               = quater / quaternionNorm;

    return quater;
}

}  // namespace SkyQuaternion

}  // namespace Boardcore
