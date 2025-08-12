/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Giovanni Annaloro
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
#include <Eigen/Core>
#include <cstdint>
#include <reflect.hpp>

namespace Boardcore
{

struct ZVKState
{
    uint64_t timestamp = 0;

    // Velocity in NED frame [m/s]
    float n = 0;
    float e = 0;
    float d = 0;

    // Acceleration in NED frame [m/s^2]
    float ax = 0;
    float ay = 0;
    float az = 0;

    // Accelerometer bias IMU0
    float bax0 = 0;
    float bay0 = 0;
    float baz0 = 0;

    // Accelerometer bias IMU1
    float bax1 = 0;
    float bay1 = 0;
    float baz1 = 0;

    // Euler angles [rad]
    float eax = 0;
    float eay = 0;
    float eaz = 0;

    // Angular velocity [rad/s]
    float avx = 0;
    float avy = 0;
    float avz = 0;

    // Gyroscope bias IMU0
    float bgx0 = 0;
    float bgy0 = 0;
    float bgz0 = 0;

    // Gyroscope bias IMU1
    float bgx1 = 0;
    float bgy1 = 0;
    float bgz1 = 0;

    ZVKState() {}

    ZVKState(uint64_t timestamp, const Eigen::Matrix<float, 24, 1>& x)
        : timestamp(timestamp), n(x(0)), e(x(1)), d(x(2)), ax(x(3)), ay(x(4)),
          az(x(5)), bax0(x(6)), bay0(x(7)), baz0(x(8)), bax1(x(9)), bay1(x(10)),
          baz1(x(11)), eax(x(12)), eay(x(13)), eaz(x(14)), avx(x(15)),
          avy(x(16)), avz(x(17)), bgx0(x(18)), bgy0(x(19)), bgz0(x(20)),
          bgx1(x(21)), bgy1(x(22)), bgz1(x(23))
    {
    }

    Eigen::Matrix<float, 24, 1> getX() const
    {
        Eigen::Matrix<float, 24, 1> x;
        x << n, e, d, ax, ay, az, bax0, bay0, baz0, bax1, bay1, baz1, eax, eay,
            eaz, avx, avy, avz, bgx0, bgy0, bgz0, bgx1,
            bgy1,  // cppcheck-suppress constStatement
            bgz1;
        return x;
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            ZVKState,
            FIELD_DEF(timestamp) FIELD_DEF(n) FIELD_DEF(e) FIELD_DEF(d)
                FIELD_DEF(ax) FIELD_DEF(ay) FIELD_DEF(az) FIELD_DEF(bax0)
                    FIELD_DEF(bay0) FIELD_DEF(baz0) FIELD_DEF(bax1)
                        FIELD_DEF(bay1) FIELD_DEF(baz1) FIELD_DEF(eax)
                            FIELD_DEF(eay) FIELD_DEF(eaz) FIELD_DEF(avx)
                                FIELD_DEF(avy) FIELD_DEF(avz) FIELD_DEF(bgx0)
                                    FIELD_DEF(bgy0) FIELD_DEF(bgz0)
                                        FIELD_DEF(bgx1) FIELD_DEF(bgy1)
                                            FIELD_DEF(bgz1));
    }
};

}  // namespace Boardcore
