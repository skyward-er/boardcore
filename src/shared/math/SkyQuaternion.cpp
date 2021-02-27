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

 // Convention used: qx, qy, qz, qw

#include "SkyQuaternion.h"
#include <kalman/ExtendedKalmanEigen.h>
#include <Eigen/Dense>
#include "iostream"
using namespace std;

SkyQuaternion::SkyQuaternion() {}

Vector4f SkyQuaternion::eul2quat(Vector3f radeul) // ZYX rotation
{
    float eulx = radeul(0);
    float euly = radeul(1);
    float eulz = radeul(2);

    float cx = cosf(eulx * 0.5F);
    float sx = sinf(eulx * 0.5F);
    float cy = cosf(euly * 0.5F);
    float sy = sinf(euly * 0.5F);
    float cz = cosf(eulz * 0.5F);
    float sz = sinf(eulz * 0.5F);

    float q1 = sx * cy * cz - cx * sy * sz;
    float q2 = cx * sy * cz + sx * cy * sz;
    float q3 = cx * cy * sz - sx * sy * cz;
    float q4 = cx * cy * cz + sx * sy * sz;

    Vector4f quat(q1, q2, q3, q4);

    return quat;
}

Vector3f SkyQuaternion::quat2eul(Vector4f quater)
{
    float q1 = quater(0);
    float q2 = quater(1);
    float q3 = quater(2);
    float q4 = quater(3);

    float eulx =
        atan2f(2.0F * (q4 * q1 + q2 * q3), 1.0F - 2.0F * (powf(q1, 2) + powf(q2, 2)));
    float euly = asinf(2.0F * (q4 * q2 - q3 * q1));
    float eulz =
        atan2f(2.0F * (q4 * q3 + q1 * q2), 1.0F - 2.0F * (powf(q2, 2) + powf(q3, 2)));

    Vector3f eul(eulx, euly, eulz);

    return eul * 180.0F / 3.14F;
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
    
    float q1;
    float q2;
    float q3;
    float q4;
        
    if (r11-r22-r33 > 0)
        q1 = 0.5 * sqrt(1+r11-r22-r33);
    else
        q1 = 0.5 * sqrt(((r32-r23)*(r32-r23) + (r12+r21)*(r12+r21) + (r31+r13)*(r31+r13)) / (3-r11+r22+r33));
        
    if (-r11+r22-r33 > 0)
        q2 = 0.5 * sqrt(1-r11+r22-r33);
    else
        q2 = 0.5 * sqrt(((r13-r31)*(r13-r31) + (r12+r21)*(r12+r21) + (r23+r32)*(r23+r32)) / (3+r11-r22+r33));
        
    if (-r11-r22+r33 > 0)
        q3 = 0.5 * sqrt(1-r11-r22+r33);
    else
        q3 = 0.5 * sqrt(((r21-r12)*(r21-r12) + (r31+r13)*(r31+r13) + (r32+r23)*(r32+r23)) / (3+r11+r22-r33));

    if (r11+r22+r33 > 0)
        q4 = 0.5 * sqrt(1+r11+r22+r33);
    else
        q4 = 0.5 * sqrt(((r32-r23)*(r32-r23) + (r13-r31)*(r13-r31) + (r21-r12)*(r21-r12)) / (3-r11-r22-r33));

    Vector4f quat(q1, q2, q3, q4);

    return quat;
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

bool SkyQuaternion::quatnormalizeEKF(VectorXf& x)
{
    Vector4f xq(x(0), x(1), x(2), x(3));
    Vector3f xp(x(4), x(5), x(6));
    Vector3f xv(x(7), x(8), x(9));
    float cd = x(10);

    float den =
        sqrtf(powf(xq(0), 2) + powf(xq(1), 2) + powf(xq(2), 2) + powf(xq(3), 2));
    if (den < 1e-8)
        return false;
    
    xq = xq / den;

    x << xq, xp, xv, cd;

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
