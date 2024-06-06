/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <Eigen/Core>
#include <catch2/catch.hpp>
#include <iostream>

#include "test-skyquaternion-data.h"

using namespace Eigen;
using namespace Boardcore;
using namespace SkyQuaternion;

void testEulerAngles(const Vector3f &actual, const Vector3f &expected)
{
    REQUIRE(actual[0] == Approx(expected[0]).margin(0.0001));
    REQUIRE(actual[1] == Approx(expected[1]).margin(0.0001));
    REQUIRE(actual[2] == Approx(expected[2]).margin(0.0001));
}

TEST_CASE("quat2stepperAngles")
{
    assert((inputs.size() == outputs.size()) &&
           "Input and Output vector sizes don't match");

    for (auto i = 0; i < inputs.size(); i++)
    {
        Vector3f euler =
            Boardcore::SkyQuaternion::quat2stepperAngles(Vector4f(inputs[i]));
        testEulerAngles(euler, outputs[i]);
    }
}