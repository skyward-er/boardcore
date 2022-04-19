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

#pragma once

#include <cmath>

namespace Boardcore
{

class Vec3
{
    friend class Mat4;
    friend class Mat3;

public:
    Vec3() { clear(); }

    Vec3(const Vec3 &rhs)
    {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
    }

    Vec3(float x, float y, float z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }

    void setX(float value) { x = value; }
    void setY(float value) { y = value; }
    void setZ(float value) { z = value; }

    void clear() { x = y = z = 0.0f; }

    float magnitude() const { return sqrt(dot(*this)); }

    bool normalize()
    {
        float l = magnitude();
        if (l <= 0)
            return false;

        (*this) *= 1.0f / l;
        return true;
    }

    void operator=(const Vec3 &rhs)
    {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
    }

    void operator*=(float value)
    {
        x *= value;
        y *= value;
        z *= value;
    }

    void operator/=(float value)
    {
        x /= value;
        y /= value;
        z /= value;
    }

    void operator+=(const Vec3 &v)
    {
        x += v.getX();
        y += v.getY();
        z += v.getZ();
    }

    void operator-=(const Vec3 &v)
    {
        x -= v.getX();
        y -= v.getY();
        z -= v.getZ();
    }

    Vec3 operator*(float t) const { return Vec3(x * t, y * t, z * t); }

    Vec3 operator/(float t) const { return Vec3(x / t, y / t, z / t); }

    Vec3 operator+(const Vec3 &v) const
    {
        return Vec3(x + v.getX(), y + v.getY(), z + v.getZ());
    }

    Vec3 operator-(const Vec3 &v) const
    {
        return Vec3(x - v.getX(), y - v.getY(), z - v.getZ());
    }

    Vec3 operator-() const { return Vec3(-x, -y, -z); }

    float dot(const Vec3 &v) const
    {
        return x * v.getX() + y * v.getY() + z * v.getZ();
    }

    Vec3 cross(const Vec3 &v) const
    {
        return Vec3(y * v.getZ() - z * v.getY(), z * v.getX() - x * v.getZ(),
                    x * v.getY() - y * v.getX());
    }

private:
    float x, y, z;
};

}  // namespace Boardcore
