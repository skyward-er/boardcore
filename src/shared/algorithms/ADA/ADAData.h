/* Copyright (c) 2018-2022 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli, Alberto Nidasio
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

#include <ostream>

namespace Boardcore
{

struct ADAState
{
    uint64_t timestamp;
    float mslAltitude;    // Altitude at mean sea level [m].
    float aglAltitude;    // Altitude above ground level [m].
    float verticalSpeed;  // Vertical speed [m/s].
    float x0;
    float x1;
    float x2;

    static std::string header()
    {
        return "timestamp,mslAltitude,aglAltitude,verticalSpeed,x0,x1,x2\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << mslAltitude << "," << aglAltitude << ","
           << verticalSpeed << "," << x0 << "," << x1 << "," << x2 << "\n";
    }
};

}  // namespace Boardcore
