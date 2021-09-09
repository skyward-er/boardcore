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

#include "SkyQuaternion.h"
#include "Constants.h"
#include "iostream"

using namespace std;

SkyQuaternion::SkyQuaternion() {}

Vector4f SkyQuaternion::eul2quat(Vector3f degeul)  // ZYX rotation
{
    float yaw = degeul(0) * DEGREES_TO_RADIANS;
    float pitch = degeul(1) * DEGREES_TO_RADIANS;
    float roll = degeul(2) * DEGREES_TO_RADIANS;

    float cyaw = cosf(yaw * 0.5F);
    float syaw = sinf(yaw * 0.5F);
    float cpitch = cosf(pitch * 0.5F);
    float spitch = sinf(pitch * 0.5F);
    float croll = cosf(roll * 0.5F);
    float sroll = sinf(roll * 0.5F);

    float qx = cyaw * cpitch * sroll - syaw * spitch * croll;
    float qy = cyaw * spitch * croll + syaw * cpitch * sroll;
    float qz = syaw * cpitch * croll - cyaw * spitch * sroll;
    float qw = cyaw * cpitch * croll + syaw * spitch * sroll;

    Vector4f quat(qx, qy, qz, qw);

    return quat;
}

Vector3f SkyQuaternion::quat2eul(Vector4f quater)
{
    float qx = quater(0);
    float qy = quater(1);
    float qz = quater(2);
    float qw = quater(3);

    float yaw = atan2f(2.0F * (qx * qy + qw * qz),
                        powf(qw, 2) + powf(qx, 2) - powf(qy, 2) - powf(qz, 2));

    float a    = -2.0F * (qx * qz - qw * qy);
    if (a > 1.0F)
    {
        a = 1.0F;
    }
    else if (a < -1.0F)
    {
        a = -1.0F;
    }

    float pitch = asinf(a);

    float roll = atan2f(2.0F * (qy * qz + qw * qx),
                        powf(qw, 2) - powf(qx, 2) - powf(qy, 2) + powf(qz, 2));

    Vector3f eul(roll, pitch, yaw);

    return eul * 180.0F / PI;
}

Vector4f SkyQuaternion::rotm2quat(Matrix3f R)
{
    float r11 = R(0, 0);
    float r12 = R(0, 1);
    float r13 = R(0, 2);
    float r21 = R(1, 0);
    float r22 = R(1, 1);
    float r23 = R(1, 2);
    float r31 = R(2, 0);
    float r32 = R(2, 1);
    float r33 = R(2, 2);

    float qx;
    float qy;
    float qz;
    float qw;

    float tr = r11 + r22 + r33;
    float r = sqrt(1 + tr);

    if (r32 - r23 > 0)
    {
        qx = 0.5 * sqrt(1 + r11 - r22 - r33);
    }
    else if (r32 - r23 < 0) 
    {
        qx = -0.5 * sqrt(1 + r11 - r22 - r33);
    }
    else
    {
        qx = 0;
    }

    if (r13 - r31 > 0)
    {
        qy = 0.5 * sqrt(1 - r11 + r22 - r33);
    }
    else if (r13 - r31 < 0) 
    {
        qy = -0.5 * sqrt(1 - r11 + r22 - r33);
    }
    else
    {
        qy = 0;
    }

    if (r21 - r12 > 0)
    {
        qz = 0.5 * sqrt(1 - r11 - r22 + r33);
    }
    else if (r21 - r12 < 0) 
    {
        qz = -0.5 * sqrt(1 - r11 - r22 + r33);
    }
    else
    {
        qz = 0;
    }

    qw = 0.5 * r;

    if (qw < 0)
    {
        qx = -qx;
        qy = -qy;
        qz = -qz;
        qw = -qw;
    }

    Vector4f quat(qx, qy, qz, qw);

    return quat / quat.norm();
}

bool SkyQuaternion::quatnormalize(Vector4f& quat)
{
    float den = sqrt(powf(quat(0), 2) + powf(quat(1), 2) + powf(quat(2), 2) +
                     powf(quat(3), 2));
    if (den < 1e-8)
        return false;

    quat = quat / den;

    return true;
}

Vector4f SkyQuaternion::quatProd(const Vector4f q1, const Vector4f q2)
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
    quater << q1w * qv2 + q2w * qv1 - qv1.cross(qv2), q1w * q2w - qv1.dot(qv2);
    float quater_norm = sqrt(quater(0) * quater(0) + quater(1) * quater(1) +
                             quater(2) * quater(2) + quater(3) * quater(3));
    quater            = quater / quater_norm;

    return quater;
}
