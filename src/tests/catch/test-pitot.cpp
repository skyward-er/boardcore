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

#include <sensors/analog/Pitot/Pitot.h>

#include <catch2/catch.hpp>
#include <functional>
#include <iostream>

using namespace Boardcore;

float totalPressures[]  = {142319.922179851, 142334.556779026, 142334.990339878,
                           142328.593255716, 142334.485137298, 142355.994981968,
                           142375.515131591, 142371.90326022,  142369.472114238,
                           142372.026452516, 142374.830116745};
float staticPressures[] = {89427.1259035819, 89399.4135799592, 89370.6155267334,
                           89337.8306466511, 89307.6055280662, 89287.1092122239,
                           89265.7025543447, 89231.9778726411, 89199.4339163267,
                           89171.8754442722, 89144.5662981688};
float results[]         = {291.616036301297, 291.754323018779, 291.863248891676,
                           291.971052285523, 292.098174588202, 292.225543982517,
                           292.351618537494, 292.469497902937, 292.585736754635,
                           292.695003991533, 292.803923124566};
constexpr float referenceTemperature = 298.15;
constexpr unsigned int SAMPLES_N     = 10;

TEST_CASE("Pitot Test")
{
    Pitot pitot(
        []()
        {
            static int i = 0;
            return totalPressures[i++];
        },
        []()
        {
            static int i = 0;
            return staticPressures[i++];
        });
    ReferenceValues references;
    references.refTemperature = referenceTemperature;

    pitot.setReferenceValues(references);

    for (int i = 0; i < SAMPLES_N; i++)
    {
        pitot.sample();
        if (pitot.getLastSample().airspeed != Approx(results[i]).epsilon(0.01))
        {
            FAIL("The computed value differs from the correct one: "
                 << pitot.getLastSample().airspeed << " != " << results[i]);
        }
    }
}