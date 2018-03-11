/* Copyright (c) 2016 Skyward Experimental Rocketry
 * Authors: Alain Carlucci
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

#ifndef MATH_MATRIX_H
#define MATH_MATRIX_H

#include "Common.h"
#include "Vec3.h"

/* TODO: double and triple check routines in these classes!!!
 *
 * This class works with matrices having the following pattern:
 * | a11 a12 a13 a14 |
 * | a21 a22 a23 a24 |
 * | a31 a32 a33 a34 |
 * |   0   0   0   1 |
 *
 * Remember that [Vec3] = [Mat4 * Vec3];
 * Where multiplication converts the Vec3 to a Vec4 like this: [x y z 1]
 */

class Mat4
{
public:
    Mat4();
    explicit Mat4(const float v[16]);

    Mat4 operator*(const Mat4 &o) const;

    Vec3 operator*(const Vec3 &vector) const;
    Vec3 transform(const Vec3 &vec) const;
    Vec3 transformInverse(const Vec3 &vec) const;

    float determinant() const;
    void setInverse(const Mat4 &m);

    float d[16];
};

class Mat3
{
public:
    Mat3();
    explicit Mat3(const float v[9]);

    Mat3 operator*(const Mat3 &o) const;

    Vec3 operator*(const Vec3 &vector) const;
    Vec3 transform(const Vec3 &vec) const;

    void setInverse(const Mat3 &m);
    void setTranspose(const Mat3 &m);

    void setVertComponents(const Vec3 &f, const Vec3 &s, const Vec3 &t);

    float d[9];
};

#endif
