/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

namespace Boardcore
{

struct NASState
{
    uint64_t timestamp;

    // 13 extended kalman states

    // Position [m]
    float n = 0;
    float e = 0;
    float d = 0;

    // Velocity [m/s]
    float vn = 0;
    float ve = 0;
    float vd = 0;

    // Attitude as quaternion
    float qx = 0;
    float qy = 0;
    float qz = 0;
    float qw = 1;

    // Gyroscope bias
    float bx = 0;
    float by = 0;
    float bz = 0;

    NASState() {}

    NASState(uint64_t timestamp, Eigen::Matrix<float, 13, 1> x)
        : timestamp(timestamp), n(x(0)), e(x(1)), d(x(2)), vn(x(3)), ve(x(4)),
          vd(x(5)), qx(x(6)), qy(x(7)), qz(x(8)), qw(x(9)), bx(x(10)),
          by(x(11)), bz(x(12))
    {
    }

    Eigen::Matrix<float, 13, 1> getX() const
    {
        return Eigen::Matrix<float, 13, 1>(n, e, d, vn, ve, vd, qx, qy, qz, qw,
                                           bx, by, bz);
    }

    static std::string header()
    {
        return "timestamp,n,e,d,vn,ve,vd,qx,qy,qz,qw,bx,by,bz\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << n << "," << e << "," << d << "," << vn << ","
           << ve << "," << vd << "," << qx << "," << qy << "," << qz << ","
           << qw << "," << bx << "," << by << "," << bz << "\n";
    }
};

}  // namespace Boardcore
