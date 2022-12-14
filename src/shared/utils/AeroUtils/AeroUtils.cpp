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

#include "AeroUtils.h"

#include <utils/Constants.h>

using namespace Eigen;
using namespace Boardcore::Constants;

namespace Boardcore
{

namespace Aeroutils
{

float relAltitude(float pressure, float pressureRef, float temperatureRef)
{
    return temperatureRef / a * (1 - powf(pressure / pressureRef, nInv));
}

float relPressure(float altitude, float pressureRef, float temperatureRef)
{
    return pressureRef * powf(1 - a * altitude / temperatureRef, n);
}

float relTemperature(float altitude, float temperatureRef)
{
    return temperatureRef - Constants::a * altitude;
}

float relDensity(float pressure, float pressureRef, float altitudeRef,
                 float temperatureRef)
{
    return pressure / (R * a * altitudeRef +
                       R * temperatureRef * powf(pressure / pressureRef, nInv));
}

float mslPressure(float pressureRef, float temperatureRef, float altitudeRef)
{
    float T0 = mslTemperature(temperatureRef, altitudeRef);
    return pressureRef / powf(1 - a * altitudeRef / T0, n);
}

float mslTemperature(float temperatureRef, float altitudeRef)
{
    return temperatureRef + Constants::a * altitudeRef;
}

float verticalSpeed(float p, float dpDt, float pRef, float tRef)
{
    return -(tRef * dpDt * powf(p / pRef, nInv)) / (a * n * p);
}

Vector2f geodetic2NED(const Vector2f& gpsData, const Vector2f& offset)
{
    float mPerDegLat = 111132.95225;
    float mPerDegLon =
        fabsf(111412.87733 * cosf(gpsData[0] * Constants::DEGREES_TO_RADIANS));

    return {
        mPerDegLat * (gpsData[0] - offset[0]),
        mPerDegLon * (gpsData[1] - offset[1]),
    };
}

}  // namespace Aeroutils

}  // namespace Boardcore
