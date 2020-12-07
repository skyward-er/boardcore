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

 // Convention used: if rad = 0 -> q4 = 1, q1 = 0, q2 = 0, q3 = 0

#include "SkyQuaternion.h"

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

    float q4 = cx * cy * cz + sx * sy * sz;
    float q1 = sx * cy * cz - cx * sy * sz;
    float q2 = cx * sy * cz + sx * cy * sz;
    float q3 = cx * cy * sz - sx * sy * cz;

    Vector4f quat(q1, q2, q3, q4);

    return quat;
}

Vector3f SkyQuaternion::quat2eul(Vector4f quat)
{
    float q4 = quat(3);
    float q1 = quat(0);
    float q2 = quat(1);
    float q3 = quat(2);

    float eulx =
        atan2f(2.0 * (q4 * q1 + q2 * q3), 1.0 - 2.0 * (powf(q1, 2) + powf(q2, 2)));
    float euly = asinf(2.0 * (q4 * q2 - q3 * q1));
    float eulz =
        atan2f(2.0 * (q4 * q3 + q1 * q2), 1.0 - 2.0 * (powf(q2, 2) + powf(q3, 2)));

    Vector3f eul(eulx, euly, eulz);

    return eul;
}

void SkyQuaternion::quatnormalize(Vector4f& quat)
{
    float den = sqrt(powf(quat(0), 2) + powf(quat(1), 2) + powf(quat(2), 2) +
                     powf(quat(3), 2));
    if (den < powf(10, -6))
    {
        den = powf(10, -5);
        quat = quat / den;
    }
    else
        quat = quat / den;
}

void SkyQuaternion::quatnormalizeEKF(VectorXf& x)
{
    Vector4f xq(x(0), x(1), x(2), x(3));
    Vector3f xp(x(4), x(5), x(6));
    Vector3f xv(x(7), x(8), x(9));
    float cd = x(10);

    float den =
        sqrt(powf(xq(0), 2) + powf(xq(1), 2) + powf(xq(2), 2) + powf(xq(3), 2));
    if (den < powf(10, -6))
    {
        den = powf(10, -5);
        xq = xq / den;
    }
    else
        xq = xq / den;

    x << xq, xp, xv, cd;
}
