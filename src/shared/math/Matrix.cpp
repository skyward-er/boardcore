/* Copyright (c) 2016 Skyward Experimental Rocketry
 * Author: Alain Carlucci
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

#include "Matrix.h"

/** Identity matrix constructor */
Mat4::Mat4()
{
    memset(d, 0, sizeof(d));
    d[0] = d[5] = d[10] = d[15] = 1;
}

/** Initialize from 4*4 float array */
Mat4::Mat4(const float v[16]) { memcpy(d, v, 16 * sizeof(float)); }

Vec3 Mat4::operator*(const Vec3 &vector) const
{
    return Vec3(vector.x * d[0] + vector.y * d[1] + vector.z * d[2] + d[3],

                vector.x * d[4] + vector.y * d[5] + vector.z * d[6] + d[7],

                vector.x * d[8] + vector.y * d[9] + vector.z * d[10] + d[11]);
}

Mat4 Mat4::operator*(const Mat4 &o) const
{
    /** Every Mat4 is supposed to be like
     * | r1 r2 r3 t1 |
     * | r4 r5 r6 t2 |
     * | r7 r8 r9 t3 |
     * |  0  0  0  1 |
     */
    Mat4 r;
    r.d[0] = (d[0] * o.d[0]) + (d[1] * o.d[4]) + (d[2] * o.d[8]);
    r.d[4] = (d[4] * o.d[0]) + (d[5] * o.d[4]) + (d[6] * o.d[8]);
    r.d[8] = (d[8] * o.d[0]) + (d[9] * o.d[4]) + (d[10] * o.d[8]);

    r.d[1] = (d[0] * o.d[1]) + (d[1] * o.d[5]) + (d[2] * o.d[9]);
    r.d[5] = (d[4] * o.d[1]) + (d[5] * o.d[5]) + (d[6] * o.d[9]);
    r.d[9] = (d[8] * o.d[1]) + (d[9] * o.d[5]) + (d[10] * o.d[9]);

    r.d[2]  = (d[0] * o.d[2]) + (d[1] * o.d[6]) + (d[2] * o.d[10]);
    r.d[6]  = (d[4] * o.d[2]) + (d[5] * o.d[6]) + (d[6] * o.d[10]);
    r.d[10] = (d[8] * o.d[2]) + (d[9] * o.d[6]) + (d[10] * o.d[10]);

    r.d[3]  = (d[0] * o.d[3]) + (d[1] * o.d[7]) + (d[2] * o.d[11]) + d[3];
    r.d[7]  = (d[4] * o.d[3]) + (d[5] * o.d[7]) + (d[6] * o.d[11]) + d[7];
    r.d[11] = (d[8] * o.d[3]) + (d[9] * o.d[7]) + (d[10] * o.d[11]) + d[11];

    return r;
}

Vec3 Mat4::transform(const Vec3 &vec) const { return (*this) * vec; }

Vec3 Mat4::transformInverse(const Vec3 &vec) const
{
    Mat4 temp;
    Vec3 tmp = vec;

    // Subtract the translation vector
    tmp.x -= d[3];
    tmp.y -= d[7];
    tmp.z -= d[11];

    // Multiply
    return Vec3(tmp.x * d[0] + tmp.y * d[4] + tmp.z * d[8],

                tmp.x * d[1] + tmp.y * d[5] + tmp.z * d[9],

                tmp.x * d[2] + tmp.y * d[6] + tmp.z * d[10]);
}

float Mat4::determinant() const
{
    return -d[8] * d[5] * d[2] + d[4] * d[9] * d[2] + d[8] * d[1] * d[6] -
           d[0] * d[9] * d[6] - d[4] * d[1] * d[10] + d[0] * d[5] * d[10];
}

void Mat4::setInverse(const Mat4 &m)
{
    assert(&m != this);
    float det = m.determinant();
    if (det == 0)
        return;

    det = ((float)1.0) / det;

    d[0] = (-m.d[9] * m.d[6] + m.d[5] * m.d[10]) * det;
    d[4] = (m.d[8] * m.d[6] - m.d[4] * m.d[10]) * det;
    d[8] = (-m.d[8] * m.d[5] + m.d[4] * m.d[9]) * det;

    d[1] = (m.d[9] * m.d[2] - m.d[1] * m.d[10]) * det;
    d[5] = (-m.d[8] * m.d[2] + m.d[0] * m.d[10]) * det;
    d[9] = (m.d[8] * m.d[1] - m.d[0] * m.d[9]) * det;

    d[2]  = (-m.d[5] * m.d[2] + m.d[1] * m.d[6]) * det;
    d[6]  = (+m.d[4] * m.d[2] - m.d[0] * m.d[6]) * det;
    d[10] = (-m.d[4] * m.d[1] + m.d[0] * m.d[5]) * det;

    d[3] = (m.d[9] * m.d[6] * m.d[3] - m.d[5] * m.d[10] * m.d[3] -
            m.d[9] * m.d[2] * m.d[7] + m.d[1] * m.d[10] * m.d[7] +
            m.d[5] * m.d[2] * m.d[11] - m.d[1] * m.d[6] * m.d[11]) *
           det;

    d[7] = (-m.d[8] * m.d[6] * m.d[3] + m.d[4] * m.d[10] * m.d[3] +
            m.d[8] * m.d[2] * m.d[7] - m.d[0] * m.d[10] * m.d[7] -
            m.d[4] * m.d[2] * m.d[11] + m.d[0] * m.d[6] * m.d[11]) *
           det;

    d[11] = (m.d[8] * m.d[5] * m.d[3] - m.d[4] * m.d[9] * m.d[3] -
             m.d[8] * m.d[1] * m.d[7] + m.d[0] * m.d[9] * m.d[7] +
             m.d[4] * m.d[1] * m.d[11] - m.d[0] * m.d[5] * m.d[11]) *
            det;
}

Mat3::Mat3()
{
    memset(d, 0, sizeof(d));
    d[0] = d[4] = d[8] = 1;
}

Mat3::Mat3(const float v[9]) { memcpy(d, v, 9 * sizeof(float)); }

Mat3 Mat3::operator*(const Mat3 &o) const
{
    Mat3 r;
    r.d[0] = d[0] * o.d[0] + d[1] * o.d[3] + d[2] * o.d[6];
    r.d[1] = d[0] * o.d[1] + d[1] * o.d[4] + d[2] * o.d[7];
    r.d[2] = d[0] * o.d[2] + d[1] * o.d[5] + d[2] * o.d[8];

    r.d[3] = d[3] * o.d[0] + d[4] * o.d[3] + d[5] * o.d[6];
    r.d[4] = d[3] * o.d[1] + d[4] * o.d[4] + d[5] * o.d[7];
    r.d[5] = d[3] * o.d[2] + d[4] * o.d[5] + d[5] * o.d[8];

    r.d[6] = d[6] * o.d[0] + d[7] * o.d[3] + d[8] * o.d[6];
    r.d[7] = d[6] * o.d[1] + d[7] * o.d[4] + d[8] * o.d[7];
    r.d[8] = d[6] * o.d[2] + d[7] * o.d[5] + d[8] * o.d[8];
    return r;
}

Vec3 Mat3::operator*(const Vec3 &vector) const
{
    return Vec3(vector.x * d[0] + vector.y * d[1] + vector.z * d[2],
                vector.x * d[3] + vector.y * d[4] + vector.z * d[5],
                vector.x * d[6] + vector.y * d[7] + vector.z * d[8]);
}

Vec3 Mat3::transform(const Vec3 &vec) const { return (*this) * vec; }

void Mat3::setInverse(const Mat3 &m)
{
    assert(&m != this);
    float t4  = m.d[0] * m.d[4];
    float t6  = m.d[0] * m.d[5];
    float t8  = m.d[1] * m.d[3];
    float t10 = m.d[2] * m.d[3];
    float t12 = m.d[1] * m.d[6];
    float t14 = m.d[2] * m.d[6];

    // Calculate the determinant
    float t16 = (t4 * m.d[8] - t6 * m.d[7] - t8 * m.d[8] + t10 * m.d[7] +
                 t12 * m.d[5] - t14 * m.d[4]);

    // Make sure the determinant is non-zero.
    if (t16 == (float)0.0f)
        return;
    float t17 = 1 / t16;

    d[0] = (m.d[4] * m.d[8] - m.d[5] * m.d[7]) * t17;
    d[1] = -(m.d[1] * m.d[8] - m.d[2] * m.d[7]) * t17;
    d[2] = (m.d[1] * m.d[5] - m.d[2] * m.d[4]) * t17;
    d[3] = -(m.d[3] * m.d[8] - m.d[5] * m.d[6]) * t17;
    d[4] = (m.d[0] * m.d[8] - t14) * t17;
    d[5] = -(t6 - t10) * t17;
    d[6] = (m.d[3] * m.d[7] - m.d[4] * m.d[6]) * t17;
    d[7] = -(m.d[0] * m.d[7] - t12) * t17;
    d[8] = (t4 - t8) * t17;
}

void Mat3::setTranspose(const Mat3 &m)
{
    d[0] = m.d[0];
    d[1] = m.d[3];
    d[2] = m.d[6];
    d[3] = m.d[1];
    d[4] = m.d[4];
    d[5] = m.d[7];
    d[6] = m.d[2];
    d[7] = m.d[5];
    d[8] = m.d[8];
}
