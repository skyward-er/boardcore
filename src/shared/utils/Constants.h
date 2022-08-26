/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Author: Alain Carlucci
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

namespace Boardcore
{

namespace Constants
{

static constexpr float PI                 = 3.14159265f;  // [rad]
static constexpr float DEGREES_TO_RADIANS = PI / 180.0f;
static constexpr float RADIANS_TO_DEGREES = 180.0f / PI;

static constexpr float g = 9.80665f;  // [m^s^2]

static constexpr float a = 0.0065f;  // Troposphere temperature gradient [deg/m]
static constexpr float R = 287.05f;  // Air gas constant [J/Kg/K]
static constexpr float n = g / (R * a);
static constexpr float nInv = (R * a) / g;

static constexpr float CO    = 340.3;  // Sound speed at ground altitude [m/s]
static constexpr float ALPHA = -3.871e-3;  // Sound speed gradient [1/s]
static constexpr float RHO_0 = 1.225;      // Air density at sea level [kg/m^3]
static constexpr float Hn    = 10400.0;    // Scale height [m]

static constexpr float MSL_PRESSURE    = 101325.0f;  // [Pa]
static constexpr float MSL_TEMPERATURE = 288.15f;    // [Kelvin]

static constexpr float B21_LATITUDE  = 45.501141;
static constexpr float B21_LONGITUDE = 9.156281;

}  // namespace Constants

}  // namespace Boardcore
