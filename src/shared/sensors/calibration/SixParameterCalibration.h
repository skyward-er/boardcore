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
 * Six-Parameter Calibration uses, for each axis, a coefficient to be multiplied
 * and a constant to be added, so that is verified the formula:
 *    x' = p1*x + q1;
 *    y' = p2*y + q2;
 *    z' = p3*z + q3;
 * Where (x, y, z) is the input and (x', y', z') the output.
 */
template <typename SensorData>
class SixParameterCorrector : public ValuesCorrector<SensorData>
{
public:
    SixParameterCorrector() : SixParameterCorrector({1, 1, 1}, {0, 0, 0}) {}

    SixParameterCorrector(const Vector3f& _p, const Vector3f& _q) : p(_p), q(_q)
    {
    }

    void setIdentity() override
    {
        p = {1, 1, 1};
        q = {0, 0, 0};
    }

    void operator>>(Matrix<float, 3, 2>& out) const
    {
        out.col(0) = p.transpose();
        out.col(1) = q.transpose();
    }

    void operator<<(const Matrix<float, 3, 2>& in)
    {
        p = in.col(0).transpose();
        q = in.col(1).transpose();
    }

    SensorData correct(const SensorData& input) const override
    {
        SensorData output;
        Vector3f vec;

        input >> vec;
        vec = p.cwiseProduct(vec) + q;
        output << vec;

        return output;
    }

private:
    Vector3f p, q;
};

template <typename SensorData>
class SixParameterCalibration
    : public AbstractCalibrationModel<SensorData, SensorData, AxisOrientation>
{
public:
    SixParameterCalibration(unsigned _maxSamples)
        : samples(_maxSamples, 6), ref(1, 0, 0), numSamples(0),
          maxSamples(_maxSamples)
    {
    }

    void setReferenceVector(Vector3f vec) { ref = vec; }
    Vector3f getReferenceVector() { return ref; }

    bool feed(const SensorData& data, const AxisOrientation& transform) override
    {
        if (numSamples >= maxSamples)
            return false;

        Vector3f measured, expected;

        data >> measured;
        expected = transform.getMatrix().transpose() * ref;

        samples.block<1, 3>(numSamples, 0) = measured.transpose();
        samples.block<1, 3>(numSamples, 3) = expected.transpose();

        numSamples++;
        return true;
    }

    ValuesCorrector<SensorData>* computeResult() override
    {
        Vector3f p, q;

        for (int i = 0; i < 3; ++i)
        {
            MatrixXf coeffs(numSamples, 2);
            Vector2f solution;

            coeffs.fill(1);
            coeffs.block(0, 0, numSamples, 1) =
                samples.block(0, i, numSamples, 1);

            solution = coeffs.colPivHouseholderQr().solve(
                samples.block(0, i + 3, numSamples, 1));

            p[i] = solution[0];
            q[i] = solution[1];
        }

        return new SixParameterCorrector<SensorData>(p, q);
    }

private:
    /*
     * The matrix contains x, y, z measured and x', y', z' expected for each
     * row. Its shape is (N x 6)
     */
    MatrixXf samples;
    Vector3f ref;
    unsigned numSamples, maxSamples;
};
