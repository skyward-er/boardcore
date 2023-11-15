/* Copyright (c) 2019-2023 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Emilio Corigliano
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

#include <utils/Constants.h>

#include <Eigen/Core>
#include <cmath>

namespace Boardcore
{

namespace Aeroutils
{

/**
 * @brief Returns the altitude given the pressure with respect to a reference
 * pressure and temperature, using International Standard Atmosphere model.
 *
 * @warning This function is valid for altitudes below 11000 meters above sea
 * level.
 * @warning This function provides a relative altitude from the altitude where
 * the reference pressure and temperature were measured. It does not provide
 * altitude above mean sea level unless the reference is, in fact, the sea
 * level.
 *
 * This means that if the reference pressure and temperature are those at mean
 * sea level, then the returned altitude is from mean sea level. Otherwise if
 * the reference pressure and temperature are those form the launchpad, then the
 * returned altitude is from above ground level.
 *
 * @param pressure Current absolute pressure [Pa]
 * @param pressureRef Pressure at reference altitude (must be > 0) [Pa]
 * @param temperatureRef Temperature at reference altitude [K]
 * @return Current altitude with respect to the reference altitude [m]
 */
float relAltitude(float pressure, float pressureRef = Constants::MSL_PRESSURE,
                  float temperatureRef = Constants::MSL_TEMPERATURE);

/**
 * @brief Returns the pressure given the altitude with respect to a reference
 * pressure and temperature, using International Standard Atmosphere model.
 *
 * @warning This function is valid for altitudes below 11000 meters above sea
 * level.
 * @warning This function provides a relative pressure at the given altitude
 * relative to the altitude where the reference pressure and temperature were
 * measured. It does not provide altitude above mean sea level unless the
 * reference is, in fact, the sea level.
 *
 * This means that if the reference pressure and temperature are those at mean
 * sea level, then the returned pressure is from mean sea level. Otherwise if
 * the reference pressure and temperature are those form the launchpad, then the
 * returned pressure is from above ground level.
 *
 * @param altitude Current relative altitude wrt the altitude of the reference
 * pressure and temperature [m]
 * @param pressureRef Pressure at reference altitude (must be > 0) [Pa]
 * @param temperatureRef Temperature at reference altitude [K]
 * @return Current pressure at the given altitude wrt the reference altitude [m]
 */
float relPressure(float altitude, float pressureRef = Constants::MSL_PRESSURE,
                  float temperatureRef = Constants::MSL_TEMPERATURE);

/**
 * @brief Returns the temperature at the given altitude with respect to the
 * reference temperature.
 *
 * @warning This function is valid for altitudes below 11000 meters above sea
 * level.
 * @warning This function provides a relative temperature from the altitude
 * where the reference temperature is measured. It does not provide temperature
 * above mean sea level unless the reference is, in fact, measured at the sea
 * level.
 *
 * This means that if the reference temperature is that at mean sea level, then
 * the returned temperature is that at the given altitude from mean sea level.
 * Otherwise if the reference temperature is that form the launchpad, then the
 * returned temperature is that at the given altitude from above ground level.
 *
 * @param altitude Current relative altitude wrt the altitude of the reference
 * pressure [m]
 * @param temperatureRef Temperature at reference altitude [K]
 * @return Current temperature at the given altitude wrt the reference altitude
 * [m]
 */
float relTemperature(float altitude,
                     float temperatureRef = Constants::MSL_TEMPERATURE);

/**
 * @brief Returns the air density given the pressure with respect to a
 * reference pressure, altitude and temperature, using the International
 * Standard Atmosphere model.
 *
 * @warning This function is valid for altitudes below 11000 meters above sea
 * level.
 *
 * @param pressure Current atmospheric pressure [Pa]
 * @param pressureRef Pressure at reference altitude (must be > 0) [Pa]
 * @param altitudeRef Reference altitude [m]
 * @param temperatureRef Temperature at reference altitude [K]
 * @return Current air density  [Kg/m^3]
 */
float relDensity(float pressure, float pressureRef = Constants::MSL_PRESSURE,
                 float altitudeRef    = 0,
                 float temperatureRef = Constants::MSL_TEMPERATURE);

/**
 * @brief Returns the expected pressure at mean sea level based on temperature
 * and pressure at a reference altitude, using International Standard Atmosphere
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
 * @brief Returns the expected temperature at mean sea level based on
 * temperature at a reference altitude, using International Standard Atmosphere
 * model.
 *
 * @warning This function is valid for altitudes below 11000 meters above sea
 * level.
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
 * @param dpDt Rate of change of pressure [Pa/s]
 * @param pRef Reference pressure (must be > 0) [Pa]
 * @param tRef Reference temperature [K]
 * @return Vertical speed, positive upwards [m/s]
 */
float verticalSpeed(float p, float dpDt, float pRef, float tRef);

/**
 * @brief Converts decimal degrees of latitude and longitude into displacement
 * in meters between two positions the with an ellipsoidal earth model.
 *
 * @param target Coordinates of target position [lat lon][deg]
 * @param origin Coordinates of the Initial position used as the origin of the
 * NED frame [lat lon][deg]
 * @return Target NED position with respect to the origin coordinates [n e][m]
 */
Eigen::Vector2f geodetic2NED(const Eigen::Vector2f& target,
                             const Eigen::Vector2f& origin);

/**
 * @brief Computes the rho (air density) of air at the given altitude.
 *
 * @param d Altitude agl in NED frame [m].
 * @param t0 Temperature at ground level [K].
 * @return The air density at the given altitude [kg/m^3].
 */
float computeRho(float d, float t0);

/**
 * @brief Computes the speed of sound at the given altitude.
 *
 * @param d Altitude agl in NED frame [m].
 * @param t0 Temperature at ground level [K].
 * @return The speed of sound at the given altitude [m/s].
 */
float computeSoundSpeed(float d, float t0);

/**
 * @brief Computes the mach relative to the speed at a certain altitude.
 *
 * @param d Altitude agl in NED frame [m].
 * @param vtot Total speed [m/s].
 * @param t0 Temperature at ground level [K].
 * @return The mach relative to the speed at a certain altitude.
 */
float computeMach(float d, float vtot, float t0);

}  // namespace Aeroutils

}  // namespace Boardcore
