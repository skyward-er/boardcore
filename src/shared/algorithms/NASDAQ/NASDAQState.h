/* Copyright (c) 2026 Skyward Experimental Rocketry
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
#include <reflect.hpp>

namespace Boardcore
{

struct NASDAQState
{
    uint64_t timestamp = 0;

    // Position [m]
    float n = 0;  ///< North (x)
    float e = 0;  ///< East  (y)
    float d = 0;  ///< Down  (z)

    // Velocity [m/s]
    float vn = 0;  ///< Velocity North (x)
    float ve = 0;  ///< Velocity East  (y)
    float vd = 0;  ///< Velocity Down  (z)

    // Covariance of the filter
    float c0 = 0;
    float c1 = 0;
    float c2 = 0;
    float c3 = 0;
    float c4 = 0;
    float c5 = 0;

    NASDAQState() {}

    NASDAQState(uint64_t timestamp, const Eigen::Matrix<float, 12, 1>& x)
        : timestamp(timestamp), n(x(0)), e(x(1)), d(x(2)), vn(x(3)), ve(x(4)),
          vd(x(5)), c0(x(6)), c1(x(7)), c2(x(8)), c3(x(9)), c4(x(10)),
          c5(x(11))
    {
    }

    Eigen::Matrix<float, 12, 1> getX() const
    {
        return Eigen::Matrix<float, 12, 1>(n, e, d, vn, ve, vd, c0, c1, c2, c3,
                                           c4, c5);
    }
    static constexpr auto reflect()
    {
        return STRUCT_DEF(NASDAQState,
                          FIELD_DEF(timestamp) FIELD_DEF(n) FIELD_DEF(e)
                              FIELD_DEF(d) FIELD_DEF(vn) FIELD_DEF(ve)
                                  FIELD_DEF(vd) FIELD_DEF(c0) FIELD_DEF(c1)
                                      FIELD_DEF(c2) FIELD_DEF(c3) FIELD_DEF(c4)
                                          FIELD_DEF(c5));
    }
};

}  // namespace Boardcore
