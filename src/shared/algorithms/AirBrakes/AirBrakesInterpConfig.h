/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio, Emilio Corigliano
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
#include "AirBrakesConfig.h"

namespace Boardcore
{

struct AirBrakesInterpConfig
{
    // Minimum altitude for the filter to consider
    float FILTER_MINIMUM_ALTITUDE;

    // Maximum altitude for the filter to consider
    float FILTER_MAXIMUM_ALTITUDE;

    // Normalized value [0-1] that represents the minimum filtering action that
    // the applied filter can do.
    float STARTING_FILTER_VALUE;

    // Altitude after which the output should be the maximum extension
    float ABK_CRITICAL_ALTITUDE;

    // The delta in altitude between consequent trajectory points
    float DZ;

    // The mass correspondent to the first trajectory
    float INITIAL_MASS;

    // The delta in mass between consequent trajectory sets
    float DM;

    // Number of steps to look forward into the reference trajectories
    uint16_t N_FORWARD;
};

}  // namespace Boardcore
