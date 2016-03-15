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

#include <Common.h>

class Vec3 {
    public:
        Vec3() { clear(); }

        Vec3(float x, float y, float z) {
            d[0] = x; 
            d[1] = y;
            d[2] = z;
        }

        float get(uint8_t i) const { 
            assert(i < 3); 
            return d[i]; 
        }

        void set(uint8_t i, float value) { 
            assert(i < 3); 
            d[i] = value;
        }

        void clear() {
            d[0] = d[1] = d[2] = 0.0f;
        }

        float magnitude() const {
            return sqrt( dot(*this) );
        }

        bool normalize() {
            float l = magnitude();
            if (l <= 0)
                return false;

            (*this) *= 1.0f/l;
            return true;
        }

        void operator*=(float value) {
            d[0] *= value;
            d[1] *= value;
            d[2] *= value;
        }

        void operator+=(const Vec3 &v) {
            d[0] += v.get(0);
            d[1] += v.get(1);
            d[2] += v.get(2);
        }

        void operator-=(const Vec3 &v) {
            d[0] -= v.get(0);
            d[1] -= v.get(1);
            d[2] -= v.get(2);
        }

        Vec3 operator*(float v) const {
            return Vec3(d[0]*v, d[1]*v, d[2]*v);
        }

        Vec3 operator+(const Vec3 &v) const {
            return Vec3(d[0]+v.get(0), d[1]+v.get(1), d[2]+v.get(2));
        }

        Vec3 operator-(const Vec3 &v) const {
            return Vec3(d[0]-v.get(0), d[1]-v.get(1), d[2]-v.get(2));
        }

        float dot(const Vec3 &v) const {
            return d[0]*v.get(0) + d[1]*v.get(1) + d[2]*v.get(2);
        }

        Vec3 cross(const Vec3 &v) const {
            return Vec3(d[1] * v.get(2) - d[2] * v.get(1),
                        d[2] * v.get(0) - d[0] * v.get(2),
                        d[0] * v.get(1) - d[1] * v.get(0));
        }

    private:
        float d[3];
};

#endif /* ifndef VEC3_H */
