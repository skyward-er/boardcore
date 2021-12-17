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

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Calibration.h"

namespace Boardcore
{

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
template <typename T>
class TwelveParameterCorrector : public ValuesCorrector<T>
{
public:
    TwelveParameterCorrector()
        : TwelveParameterCorrector(Eigen::Matrix3f::Identity(), {0, 0, 0})
    {
    }

    TwelveParameterCorrector(const Eigen::Matrix3f& _w,
                             const Eigen::Vector3f& _b)
        : w(_w), b(_b)
    {
    }

    TwelveParameterCorrector(const Eigen::Matrix<float, 3, 4>& mat)
    {
        operator<<(mat);
    }

    void operator<<(const Eigen::Matrix<float, 3, 4>& rhs)
    {
        w = rhs.block<3, 3>(0, 0);
        b = rhs.col(3).transpose();
    }

    void operator>>(Eigen::Matrix<float, 3, 4>& rhs) const
    {
        rhs.block<3, 3>(0, 0) = w;
        rhs.col(3)            = b.transpose();
    }

    void setIdentity() override
    {
        w.setIdentity();
        b.setZero();
    }

    T correct(const T& input) const
    {
        T output;
        Eigen::Vector3f vec;

        input >> vec;
        vec = w * vec + b;
        output << vec;

        return output;
    }

private:
    Eigen::Matrix3f w;
    Eigen::Vector3f b;
};

template <typename T, unsigned MaxSamples>
class TwelveParameterCalibration
    : public AbstractCalibrationModel<T, TwelveParameterCorrector<T>,
                                      AxisOrientation>
{
public:
    TwelveParameterCalibration() : samples(), numSamples(0), ref({1, 0, 0}) {}

    void setReferenceVector(Eigen::Vector3f vec) { ref = vec; }
    Eigen::Vector3f getReferenceVector() { return ref; }

    bool feed(const T& data, const AxisOrientation& transform) override
    {
        if (numSamples == MaxSamples)
            return false;

        Eigen::Vector3f expected, measured;
        data >> measured;
        expected = transform.getMatrix().transpose() * ref;

        samples.row(numSamples) << measured.transpose(), 1,
            expected.transpose();
        numSamples++;

        return true;
    }

    TwelveParameterCorrector<T> computeResult() override
    {
        Eigen::Matrix<float, 3, 4> solutions;
        Eigen::MatrixXf coeffs = samples.block(0, 0, numSamples, 4);
        auto qr                = coeffs.colPivHouseholderQr();

        for (int i = 0; i < 3; ++i)
        {
            Eigen::VectorXf terms = samples.block(0, 4 + i, numSamples, 1);
            Eigen::Vector4f sol   = qr.solve(terms);
            solutions.col(i)      = sol.head<3>();
            solutions(i, 3)       = sol[3];
        }

        return {solutions};
    }

private:
    /*
     * The matrix contains x, y, z measured and x', y', z' expected for
     * each sample. Between (x, y, z) and (x', y', z') there is a column of 1s
     * Its shape is (N x 7)
     */
    Eigen::Matrix<float, MaxSamples, 7> samples;
    unsigned numSamples;
    Eigen::Vector3f ref;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace Boardcore
