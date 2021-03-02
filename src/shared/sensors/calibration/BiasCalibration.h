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

using namespace Eigen;

/**
 * This is the dumbest type of calibration possible: it stores a 3d vector
 * (called "bias") that will be added to every measurement.
 * During the calibration phase it will use a given reference vector (for
 * example the gravitational acceleration for the accelerometer), and every time
 * you'll feed the model with a new value, you have to give it the orientation
 * of the sensor, so it can guess the bias.
 */
template <typename SensorData>
class BiasCorrector : public ValuesCorrector<SensorData>
{

public:
    BiasCorrector() : bias(0, 0, 0) {}
    BiasCorrector(const Vector3f& _bias) : bias(_bias) {}

    void operator>>(Vector3f& rhs) { rhs = bias; }

    void operator<<(const Vector3f& rhs) { bias = rhs; }

    void setIdentity() override { bias = {0, 0, 0}; }

    SensorData correct(const SensorData& data) const override
    {
        SensorData out;
        Vector3f tmp;

        data >> tmp;
        tmp += bias;
        out << tmp;
        return out;
    }

private:
    Vector3f bias;
};

template <typename SensorData>
class BiasCalibration
    : public AbstractCalibrationModel<SensorData, SensorData, AxisOrientation>
{
public:
    BiasCalibration() : ref(1, 0, 0), sum(0, 0, 0), numSamples(0) {}

    void setReferenceVector(Vector3f vec) { ref = vec; }
    Vector3f getReferenceVector() { return ref; }

    bool feed(const SensorData& measured,
              const AxisOrientation& transform) override
    {
        Vector3f vec;
        measured >> vec;

        sum += (transform.getMatrix().transpose() * ref) - vec;
        numSamples++;

        return true;
    }

    ValuesCorrector<SensorData>* computeResult()
    {
        if (numSamples == 0)
            return new BiasCorrector<SensorData>({0, 0, 0});
        return new BiasCorrector<SensorData>(sum / numSamples);
    }

private:
    Vector3f sum, ref;
    unsigned numSamples;
};
