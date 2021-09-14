/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Riccardo Musso
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

/* This header file defines operators << and >> from some types
 * of SensorData.
 *
 * This way you can write for example:
 *
 * GenericSensorData data;
 * data << Vector3f{0.f, 0.f, 0.f};
 *
 * Vector3f vec;
 * data >> vec;
 *
 * That works if GenericSensorData is any of AccelerometerData, GyroscopeData
 * and MagnetometerData
 */

#pragma once

#include <Eigen/Core>

#include "sensors/SensorData.h"

using namespace Eigen;

void operator<<(AccelerometerData& lhs, const Vector3f& rhs);

void operator<<(GyroscopeData& lhs, const Vector3f& rhs);

void operator<<(MagnetometerData& lhs, const Vector3f& rhs);

void operator>>(const AccelerometerData& lhs, Vector3f& rhs);

void operator>>(const GyroscopeData& lhs, Vector3f& rhs);

void operator>>(const MagnetometerData& lhs, Vector3f& rhs);

