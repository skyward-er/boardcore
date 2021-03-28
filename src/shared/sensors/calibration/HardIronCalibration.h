/* Copyright (c) 2021 Skyward Experimental Rocketry
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
 * The Hard-iron calibration removes the bias due to the so named Hard-Iron
 * distortion of the magnetic field. Unlike bias calibration, 6-parameter
 * calibration, etc., hard-iron calibration can be applied only to magnetometer
 * samples.
 */
class HardIronCorrector : public ValuesCorrector<MagnetometerData>
{
public:
    HardIronCorrector() : HardIronCorrector({0, 0, 0}) {}

    HardIronCorrector(const Vector3f& _bias) : bias(_bias) {}

    void setIdentity() override { bias = {0, 0, 0}; }

    void operator>>(Vector3f& rhs) { rhs = bias; }

    void operator<<(const Vector3f& rhs) { bias = rhs; }

    MagnetometerData correct(const MagnetometerData& input) const override
    {
        MagnetometerData output;
        Vector3f vec;

        input >> vec;
        vec += bias;
        output << vec;

        return output;
    }

private:
    Vector3f bias;
};

template <unsigned MaxSamples>
class HardIronCalibration
    : public AbstractCalibrationModel<MagnetometerData, HardIronCorrector>
{
public:
    HardIronCalibration() : samples(), numSamples(0) {}

    bool feed(const MagnetometerData& data) override
    {
        if (numSamples >= MaxSamples)
            return false;

        Vector3f vec;
        data >> vec;

        samples.block(numSamples, 0, 1, 3) = vec.transpose();
        samples(numSamples, 3)             = 1;
        samples(numSamples, 4) =
            vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];

        numSamples++;
        return true;
    }

    HardIronCorrector computeResult() override
    {
        Vector4f sol;
        Vector3f bias;

        auto colPiv = samples.block(0, 0, numSamples, 4).colPivHouseholderQr();
        sol         = colPiv.solve(samples.block(0, 4, numSamples, 1));

        bias[0] = -sol[0] / 2;
        bias[1] = -sol[1] / 2;
        bias[2] = -sol[2] / 2;

        return {bias};
    }

private:
    /*
     * The matrix contains x, y, z measured, a column of 1s and x^2+y^2+z^2
     * row. Its shape is (N x 5)
     */
    Matrix<float, MaxSamples, 5> samples;
    unsigned numSamples;
};

