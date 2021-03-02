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
#include <Eigen/Geometry>

#include "Calibration.h"

using namespace Eigen;

/**
 * This type of correction uses a 3x3 matrix multiplied by the input
 * data and adds a 3d vector bias, so that:
 *
 *   | x' |   | w11 w12 w13 |   | x |   | b1 |
 *   | y' | = | w21 w22 w23 | . | y | + | b2 |
 *   | z' |   | w31 w32 w33 |   | z |   | b3 |
 *
 * Where (x, y, z) is the input and (x', y', z') the output.
 */
template <typename SensorData>
class TwelveParameterValuesCorrector : public ValuesCorrector<SensorData>
{
public:
    TwelveParameterValuesCorrector()
        : TwelveParameterValuesCorrector(Matrix3f::Identity(), {0, 0, 0})
    {
    }

    TwelveParameterValuesCorrector(const Matrix3f& _w, const Vector3f& _b)
        : w(_w), b(_b)
    {
    }

    TwelveParameterValuesCorrector(const Matrix<float, 3, 4>& mat){
        operator<<(mat);
    }

    void operator<<(const Matrix<float, 3, 4>& rhs)
    {
        w = rhs.block<3, 3>(0, 0);
        b = rhs.col(3);
    }

    void operator>>(Matrix<float, 3, 4>& rhs) const
    {
        rhs.block<3, 3>(0, 0) = w;
        rhs.col(3)            = b;
    }

    void setIdentity() override
    {
        w.setIdentity();
        b.setZero();
    }

    SensorData correct(const SensorData& input) const
    {
        SensorData output;
        Vector3f vec;

        input >> vec;
        vec = w * vec + b;
        output << vec;
        return output;
    }

private:
    Matrix3f w;
    Vector3f b;
};

template <typename SensorData, unsigned MaxSamples>
class TwelveParameterCalibration
    : public AbstractCalibrationModel<SensorData, SensorData, AxisOrientation>
{
public:
    TwelveParameterCalibration() : ref(1, 0, 0), numSamples(0) {}

    void setReferenceVector(Vector3f vec) { ref = vec; }
    Vector3f getReferenceVector() { return ref; }

    bool feed(const SensorData& measured,
              const AxisOrientation& transform) override
    {
        if (numSamples == MaxSamples)
            return false;

        Vector3f expected       = transform.getMatrix().transpose() * ref;
        samples.row(numSamples) = measured, 1, expected;
        numSamples++;
    }

    ValuesCorrector<SensorData>* computeResult() override
    {
        Matrix<float, 3, 4> solutions;
        auto qr = samples.block(0, 0, numSamples, 4).colPivHouseholderQr();

        for (int i = 0; i < 3; ++i)
            solutions.row(i) = qr.solve(samples.block(0, 6, numSamples, 1));

        return new TwelveParameterValuesCorrector<SensorData>(solutions);
    }

private:
    /*
     * The matrix contains x, y, z measured and x', y', z' expected for each
     * sample. Between (x, y, z) and (x', y', z') there is a column of 1s
     */
    Matrix<float, MaxSamples, 7> samples;
    Vector3f ref;
    unsigned numSamples;
};
