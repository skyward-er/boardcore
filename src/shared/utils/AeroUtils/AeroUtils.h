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

namespace Aeroutils
{

/**
 * Returns the current altitude with respect to a reference altitude for the
 * given pressure, using International Standard Atmosphere model.
 *
 * @warning This function is valid for altitudes below 11000 meters above sea
 * level
 * @warning This function provides a relative altitude from the reference
 * altitude. It does not provide altitude above mean sea level unless the
 * reference is, in fact, the sea level.
 *
 * @param pressure Current pressure [Pas]
 * @param pressureRef Pressure at reference altitude (must be > 0) [Pa]
 * @param temperatureRef Temperature at reference altitude [K]
 * @return Current altitude with respect to the reference altitude [m]
 */
float relAltitude(float pressure, float pressureRef, float temperatureRef);

/**
 * Returns the current air density with respect to a reference density and
 * temperature, using the Internation Standard Atmosphere model.
 *
 * @param pressure Current atmospheric pressure [Pa]
 * @param pressureRef Pressure at reference altitude (must be > 0) [Pa]
 * @param altitudeRef Reference altitude [m]
 * @param temperatureRef Temperature at reference altitude [K]
 * @return Current air density  [Kg/m^3]
 */
float relDensity(float pressure, float pressureRef, float altitudeRef,
                 float temperatureRef);

/**
 * Returns the expected pressure at mean sea level based on temperature and
 * pressure at a reference altitude, using International Standard Atmosphere
 * model.
 *
 * @warning This function is valid for altitudes below 11000 meters above sea
 * level
 *
 * @param pressureRef Pressure at reference altitude [Pa]
 * @param temperatureRef Temperature at reference altitude. Must be > 0 [K]
 * @param altitudeRef Reference altitude [m]
 * @return Pressure at mean sea level [Pa]
 */
float mslPressure(float pressureRef, float temperatureRef, float altitudeRef);

/**
 * Returns the expected temperature at mean sea level based on temperature at a
 * reference altitude, using International Standard Atmosphere model.
 *
 * @warning This function is valid for altitudes below 11000 meters above sea
 * level
 *
 * @param temperatureRef Temperature at reference altitude [K]
 * @param altitudeRef Reference altitude [m]
 * @return Temperature at mean sea level [K]
 */
float mslTemperature(float temperatureRef, float altitudeRef);

/**
 * Returns the vertical speed (or rate of climb) of the rocket, assuming an
 * International Standard Atmosphere model.
 *
 * @warning This function is valid for altitudes below 11000 meters above sea
 * level
 *
 * @param p Current pressure (must be > 0) [Pa]
 * @param dpDt [Rate of change of pressure [Pa/s]]
 * @param pRef Reference pressure (must be > 0) [Pa]
 * @param tRef Reference temperature [K]
 * @return Vertical speed, positive upwards [m/s]
 */
float verticalSpeed(float p, float dpDt, float pRef, float tRef);

}  // namespace Aeroutils

}  // namespace Boardcore
