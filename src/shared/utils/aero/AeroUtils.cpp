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

namespace aeroutils
{

float relAltitude(float pressure, float pressure_ref, float temperature_ref)
{
    using namespace constants;

    return temperature_ref / a * (1 - powf(pressure / pressure_ref, n_inv));
}

float relDensity(float pressure, float pressure_ref, float altitude_ref,
                 float temperature_ref)
{
    using namespace constants;

    return pressure /
           (R * a * altitude_ref +
            R * temperature_ref * powf(pressure / pressure_ref, n_inv));
}

float mslPressure(float pressure_ref, float temperature_ref, float altitude_ref)
{
    using namespace constants;
    float T0 = mslTemperature(temperature_ref, altitude_ref);

    return pressure_ref / powf(1 - a * altitude_ref / T0, n);
}

float mslTemperature(float temperature_ref, float altitude_ref)
{
    return temperature_ref + (altitude_ref * constants::a);
}

float verticalSpeed(float p, float dp_dt, float p_ref, float t_ref)
{
    using namespace constants;

    return -(t_ref * dp_dt * powf(p / p_ref, n_inv)) / (a * n * p);
}

}  // namespace aeroutils