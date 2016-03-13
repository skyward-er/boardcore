/* Vector3 
 *
 * Copyright (c) 2016 Skyward Experimental Rocketry
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
#ifndef VEC3_H
#define VEC3_H 

class Vec3 {
    public:
        Vec3() : x(0), y(0), z(0) {}
        Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

        float getX() const { return x; }
        float getY() const { return y; }
        float getZ() const { return z; }

        float clear() {
            x = y = z = 0;
        }

        float magnitude() const {
            return sqrt(x*x + y*y + z*z);
        }

        bool normalize() {
            float l = magnitude();
            if (l <= 0)
                return false;

            (*this) *= 1.0f/l;
            return true;
        }

        void operator*=(float value) {
            x *= value;
            y *= value;
            z *= value;
        }

        void operator+=(const Vec3 &v) {
            x += v.x;
            y += v.y;
            z += v.z;
        }

        void operator-=(const Vec3 &v) {
            x -= v.x;
            y -= v.y;
            z -= v.z;
        }

        Vec3 operator*(float v) const {
            return Vec3(x*v, y*v, z*v);
        }

        Vec3 operator+(const Vec3 &v) const {
            return Vec3(x+v.x, y+v.y, z+v.z);
        }

        Vec3 operator-(const Vec3 &v) const {
            return Vec3(x-v.x, y-v.y, z-v.z);
        }

        float dot(const Vec3 &v) const {
            return x*v.x + y*v.y + z*v.z;
        }

        Vec3 cross(const Vec3 &v) const {
            return Vec3(y * v.z - z * v.y,
                        z * v.x - x * v.z,
                        x * v.y - y * v.x);
        }

    private:
        float x, y, z;
};

#endif /* ifndef VEC3_H */
