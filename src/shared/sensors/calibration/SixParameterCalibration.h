/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Riccardo Musso
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <Eigen/Core>

#include "Calibration.h"
#include "sensors/SensorData.h"

/*
 * Six-Parameter Calibration uses, for each axis, a coefficient to be multiplied 
 * and a constant to be added, so that is verified the formula:
 *    x' = p1*x + q1;
 *    y' = p2*y + q2;
 *    z' = p3*z + q3;
 * Where (x, y, z) is the input and (x', y', z') the output.
*/
template <typename SensorData, size_t MaxSamples>
class SixParameterCalibration
    : public AbstractCalibrationModel<Vector3f, SensorData, AxisOrientation>
{
public:
    SixParameterCalibration() : p(0.f, 0.f, 0.f), q(0.f, 0.f, 1.f) {}

    void store(Vector3f& out) const override { out = bias; }

    void load(const Vector3f& in) override { bias = in; }

    void resetToIdentity() override { bias = {0.f, 0.f, 0.f}; }

    void startCalibrationStage() override
    {
        specificInit();

        sum        = {0.f, 0.f, 0.f};
        numSamples = 0;
    }

    void feed(const SensorData& measured,
              const AxisOrientation& transform) override
    {
        Vector3f vec;
        measured >> vec;

        sum += (transform.getMatrix().transpose() * ref) - vec;
        numSamples++;
    }

    void endCalibrationStage() override { bias = sum / numSamples; }

    SensorData correct(const SensorData& input) const override
    {
        SensorData output;
        Vector3f vec;

        input >> vec;
        vec += bias;
        output << vec;

        return output;
    }

private:
    Vector3f bias, sum, ref;
    unsigned numSamples;

    void specificInit() {}
};

template <>
void SixParameterCalibration<AccelerometerData>::specificInit()
{
    setReferenceVector({0.f, 0.f, -1.f});
}
