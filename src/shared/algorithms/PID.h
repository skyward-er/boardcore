/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Vincenzo Santomarco
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

#include <limits>

namespace Boardcore
{

/**
 * @brief Proportional and integral controller with saturation.
 * */
class PIController
{

public:
    PIController(double Kp, double Ki,
                 double uMin = -std::numeric_limits<double>::infinity(),
                 double uMax = std::numeric_limits<double>::infinity())
        : Kp(Kp), Ki(Ki), uMin(uMin), uMax(uMax)
    {
    }

    /**
     * @brief Update the PI internal state.
     * */
    double update(double error)
    {

        double p = Kp * error;

        if (!saturation)
        {
            i = i + Ki * error;
        }

        double u = p + i;

        return antiWindUp(u);
    }

    /**
     * @brief Anti-windup mechanism.
     */
    double antiWindUp(double u)
    {
        if (u < uMin)
        {
            u          = uMin;
            saturation = true;
        }
        else if (u > uMax)
        {
            u          = uMax;
            saturation = true;
        }
        else
        {
            saturation = false;
        }

        return u;
    }

private:
    const double Kp;          // Proportional factor.
    const double Ki;          // Integral factor.
    const double uMin, uMax;  // Anti-windup limits.
    double i = 0;             // Integral contribution.

    bool saturation = false;
};

}  // namespace Boardcore
