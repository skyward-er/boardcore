/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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

class ExtendedKalmanState
{
    float n, e, d, vn, ve, vd, qx, qy, qz, qw, b1, b2, b3;

    ExtendedKalmanState(Eigen::Matrix<float, 13, 1> x)
        : n(x(0)), e(x(1)), d(x(2)), vn(x(3)), ve(x(4)), vd(x(5)), qx(x(6)),
          qy(x(7)), qz(x(8)), qw(x(9)), b1(x(10)), b2(x(11)), b3(x(12))
    {
    }

    static std::string header()
    {
        return "n,e,d,vn,ve,vd,qx,qy,qz,qw,b1,b2,b3,b4\n";
    }

    void print(std::ostream& os) const
    {
        os << (0) << "," << (1) << "," << (2) << "," << (3) << "," << (5) << ","
           << (5) << "," << (6) << "," << (7) << "," << (8) << "," << (9) << ","
           << (10) << "," << (11) << "," << (12) << "\n";
    }
};

}  // namespace Boardcore
