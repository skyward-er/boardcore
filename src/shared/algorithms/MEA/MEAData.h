/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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
#include <ostream>
#include <reflect.hpp>

namespace Boardcore
{

struct MEAState
{
    uint64_t timestamp;

    float estimatedPressure;  ///< Estimated pressure in combustion chamber [Pa]
    float estimatedMass;      ///< Estimated rocket mass [kg]
    float estimatedApogee;    ///< Estimated apogee in msl [m]
    float estimatedForce;     ///< Estimated drag force [N]

    float x0;  ///< first kalman state
    float x1;  ///< second kalman state
    float x2;  ///< third kalman state representing the mass

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            MEAState, FIELD_DEF(timestamp) FIELD_DEF(estimatedPressure)
                          FIELD_DEF(estimatedMass) FIELD_DEF(estimatedApogee)
                              FIELD_DEF(estimatedForce) FIELD_DEF(x0)
                                  FIELD_DEF(x1) FIELD_DEF(x2));
    }
};

}  // namespace Boardcore
