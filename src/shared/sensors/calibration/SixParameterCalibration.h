/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#pragma once

#include <Common.h>

#include <Eigen/Core>
#include <iostream>

#include "Calibration.h"
#include "sensors/SensorData.h"

/*
 * Six-Parameter Calibration uses, for each axis, a coefficient to be multiplied
 * and a constant to be added, so that is verified the formula:
 *
 *    x' = p1*x + q1;
 *    y' = p2*y + q2;
 *    z' = p3*z + q3;
 *
 * Where (x, y, z) is the input and (x', y', z') the output.
 */
template <typename T>
class SixParameterCorrector : public ValuesCorrector<T>
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

    T correct(const T& input) const override
    {
        T output;
        Vector3f vec;

        input >> vec;
        vec = p.cwiseProduct(vec) + q;
        output << vec;

        return output;
    }

private:
    Vector3f p, q;
};

template <typename T, unsigned MaxSamples>
class SixParameterCalibration
    : public AbstractCalibrationModel<T, SixParameterCorrector<T>,
                                      AxisOrientation>
{
public:
    SixParameterCalibration() : samples(), ref(1, 0, 0), numSamples(0) {}

    void setReferenceVector(Vector3f vec) { ref = vec; }
    Vector3f getReferenceVector() { return ref; }

    bool feed(const T& data, const AxisOrientation& transform) override
    {
        if (numSamples >= MaxSamples)
            return false;

        Vector3f measured, expected;

        data >> measured;
        expected = transform.getMatrix() * ref;
        // std::cout << "Expected: " << expected.transpose() << "\n";
        // std::cout << "Got: " << measured.transpose() << "\n";

        /*
         * measered and expected are column vectors, we need to traspose them
         * to be row vectors
         */
        samples.block(numSamples, 0, 1, 3) = measured.transpose();
        samples.block(numSamples, 3, 1, 3) = expected.transpose();

        numSamples++;
        return true;
    }

    SixParameterCorrector<T> computeResult() override
    {
        Vector3f p, q;

        for (int i = 0; i < 3; ++i)
        {
            MatrixXf coeffs(numSamples, 2);
            VectorXf terms = samples.block(0, i + 3, numSamples, 1);

            Vector2f solution;
            coeffs.fill(1);
            coeffs.block(0, 0, numSamples, 1) =
                samples.block(0, i, numSamples, 1);

            auto solver = coeffs.colPivHouseholderQr();
            solution    = solver.solve(terms);

            p[i] = solution[0];
            q[i] = solution[1];
        }

        return {p, q};
    }

private:
    /*
     * The matrix contains x, y, z measured and x', y', z' expected for each
     * row. Its shape is (N x 6)
     */
    Matrix<float, MaxSamples, 6> samples;
    Vector3f ref;
    unsigned numSamples;

public:
    /*
     * Only needed for test-calibration.cpp entry point
     */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
