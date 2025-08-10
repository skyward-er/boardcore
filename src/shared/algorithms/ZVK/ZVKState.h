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
#include <cstdint>
#include <Eigen/Core>
#include <reflect.hpp>

namespace Boardcore 
{

struct ZVKState
{
    uint64_t timestamp = 0;

    //13 extended kalman states

    //Attitude as quaternion body to NED frame
    float qx = 0;
    float qy = 0;
    float qz = 0;
    float qw = 0;

    //Velocity in NED frame [m/s]
    float vn = 0;
    float ve = 0;
    float vd = 0;

    //Position [m]
    float n = 0;
    float e = 0;
    float d = 0;

    //Accelerometer bias 
    float bax = 0;
    float bay = 0;
    float baz = 0;

    //Gyroscope bias 
    float bgx = 0;
    float bgy = 0;
    float bgz = 0;

    ZVKState(){}

    ZVKState(uint64_t timestamp, const Eigen::Matrix<float, 13 ,1>& x) 
        : timestamp(timestamp), qx(x(0)), qy(x(1)), qz(x(2)), qw(x(3)), vn(x(4)), ve(x(5)), vd(x(6)),
        bax(x(7)), bay(x(8)), baz(x(9)), bgx(x(10)), bgy(x(11)), bgz(x(12))
    {}

    Eigen::Matrix<float, 16, 1> getX() const 
    {
        return Eigen::Matrix<float, 13, 1>(qx, qy, qz, qw, vn, ve, vd, n, e, d,  bax, bay, baz, bgx, bgy, bgz);
    }

        static constexpr auto reflect()
    {
        return STRUCT_DEF(ZVKState,
                          FIELD_DEF(timestamp) FIELD_DEF(qx) FIELD_DEF(qy)
                              FIELD_DEF(qz) FIELD_DEF(qw) FIELD_DEF(vn)
                                  FIELD_DEF(ve) FIELD_DEF(vd) FIELD_DEF(n)
                                    FIELD_DEF(e) FIELD_DEF(d) FIELD_DEF(bax)
                                        FIELD_DEF(bay) FIELD_DEF(baz) FIELD_DEF(bgx)
                                            FIELD_DEF(bgy) FIELD_DEF(bgz));
    }

};

} //namespace Boardcore