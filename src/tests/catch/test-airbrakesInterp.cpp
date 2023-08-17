/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <algorithms/AirBrakes/AirBrakesInterp.h>

#include <catch2/catch.hpp>
#include <iostream>

#include "../algorithms/Airbrakes/test-airbrakesInterp-data.h"

using namespace Boardcore;

constexpr float MINIMUM_ALTITUDE      = 1000;
constexpr float STARTING_FILTER_VALUE = 0.9f;
constexpr float DZ                    = 10;
constexpr float INITIAL_MASS          = 28;
constexpr float DM                    = 0.2f;

AirBrakesInterpConfig getConfig()
{
    AirBrakesInterpConfig config;
    config.MINIMUM_ALTITUDE      = MINIMUM_ALTITUDE;
    config.STARTING_FILTER_VALUE = STARTING_FILTER_VALUE;
    config.DZ                    = DZ;
    config.INITIAL_MASS          = INITIAL_MASS;
    config.DM                    = DM;
    return config;
}
