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

#include <algorithms/ReferenceValues.h>

namespace Common
{

namespace ReferenceConfig
{

#if defined(EUROC)

static const Boardcore::ReferenceValues defaultReferenceValues = {
    160.0,      // [m] Altitude
    99418.0,    // [Pa] Pressure
    288.15,     // [K] Temperature
    39.389733,  // [deg] Start latitude
    -8.288992,  // [deg] Start longitude
    Boardcore::Constants::MSL_PRESSURE,
    Boardcore::Constants::MSL_TEMPERATURE,
};

const Eigen::Vector3f nedMag(0.5939, -0.0126, 0.8044);

#elif defined(ROCCARASO)

static const Boardcore::ReferenceValues defaultReferenceValues = {
    1414.0,      // [m] Altitude
    85452.0,     // [Pa] Pressure
    278.95,      // [K] Temperature
    41.8089005,  // [deg] Start latitude
    14.0546716,  // [deg] Start longitude
    Boardcore::Constants::MSL_PRESSURE,
    Boardcore::Constants::MSL_TEMPERATURE,
};

const Eigen::Vector3f nedMag(0.5244, 0.0368, 0.8507);

#else  // Milan

static const Boardcore::ReferenceValues defaultReferenceValues = {
    135.0,              // [m] Altitude
    99714.0,            // [Pa] Pressure
    278.27,             // [K] Temperature
    45.50106793771145,  // [deg] Start latitude
    9.156376900740167,  // [deg] Start longitude
    Boardcore::Constants::MSL_PRESSURE,
    Boardcore::Constants::MSL_TEMPERATURE,
};

const Eigen::Vector3f nedMag(0.4732, 0.0272, 0.8805);

#endif

}  // namespace ReferenceConfig

}  // namespace Common
