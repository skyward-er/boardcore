/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <cmath>

namespace Boardcore
{

namespace aeroutils
{

namespace constants
{

// Troposphere temperature gradient [deg/m]
constexpr float a = 0.0065f;

// Acceleration of gravity [m^s^2]
constexpr float g = 9.80665f;

// Air gas constant [J/(Kg*K])]
constexpr float R = 287.05f;

constexpr float n     = g / (R * a);
constexpr float n_inv = (R * a) / g;

}  // namespace constants

/**
 * Returns the current altitude with respect to a reference altitude for the
 * given pressure, using International Standard Atmosphere  model.
 *
 * @warning This function is valid for altitudes below 11000 meters above sea
 * level
 * @warning This function provides a relative altitude from the reference
 * altitude. It does not provide altitude above mean sea level unless the
 * reference is, in fact, the sea level.
 *
 * @param pressure Current pressure [Pascal]
 * @param pressure_ref Pressure at reference altitude (must be > 0) [Pascal]
 * @param temperature_ref Temperature at reference altitude [Kelvin]
 * @return Current altitude with respect to the reference altitude [meters]
 */
float relAltitude(float pressure, float pressure_ref, float temperature_ref);

/**
 * Returns the current air density with respect to a reference density and
 * temperature, using the Internation Standard Atmosphere model.
 *
 * @param pressure Current atmospheric pressure [Pascal]
 * @param pressure_ref Pressure at reference altitude (must be > 0) [Pascal]
 * @param altitude_ref Reference altitude [m]
 * @param temperature_ref Temperature at reference altitude [Kelvin]
 * @return Current air density  [Kg/m^3]
 */
float relDensity(float pressure, float pressure_ref, float altitude_ref,
                 float temperature_ref);

/**
 * Returns the expected pressure at mean sea level based on temperature and
 * pressure at a reference altitude, using International Standard Atmosphere
 * model.
 *
 * @warning This function is valid for altitudes below 11000 meters above sea
 * level
 *
 * @param pressure_ref Pressure at reference altitude [Pascal]
 * @param temperature_ref Temperature at reference altitude. Must be > 0
 * [Kelvin]
 * @param altitude_ref Reference altitude [meters]
 * @return Pressure at mean sea level [pascal]
 */
float mslPressure(float pressure_ref, float temperature_ref,
                  float altitude_ref);

/**
 * Returns the expected temperature at mean sea level based on temperature at a
 * reference altitude, using International Standard Atmosphere model.
 *
 * @warning This function is valid for altitudes below 11000 meters above sea
 * level
 *
 * @param temperature_ref Temperature at reference altitude [Kelvin]
 * @param altitude_ref Reference altitude [meters]
 * @return Temperature at mean sea level [Kelvin]
 */
float mslTemperature(float temperature_ref, float altitude_ref);

/**
 * Returns the vertical speed (or rate of climb) of the rocket, assuming an
 * International Standard Atmosphere model.
 *
 * @warning This function is valid for altitudes below 11000 meters above sea
 * level
 *
 * @param p Current pressure (must be > 0) [Pa]
 * @param dp_dt [Rate of change of pressure [Pa/s]]
 * @param p_ref Reference pressure (must be > 0) [Pa]
 * @param t_ref Reference temperature [K]
 * @return Vertical speed, positive upwards [m/s]
 */
float verticalSpeed(float p, float dp_dt, float p_ref, float t_ref);

}  // namespace aeroutils

}  // namespace Boardcore
