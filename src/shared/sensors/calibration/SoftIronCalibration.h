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
 * The Soft-iron calibration removes the measurement error given by both the Hard and Soft Iron distortion of the magnetic field.
 */
class SoftIronCorrector : public ValuesCorrector<MagnetometerData>
{
public:
    SoftIronCorrector() : SoftIronCorrector({ 1, 1, 1 }, { 0, 0, 0 }): {}

    SoftIronCorrector(const Vector3f& _p, const Vector3f& _q) : p(_p), q(_q)
    {
    }

    void setIdentity() override
    {
        p = { 1, 1, 1 };
        q = { 0, 0, 0 };
    }

    void operator>>(Matrix<float, 3, 2>& rhs)
    {
        rhs.col(0) = p.transpose();
        rhs.col(1) = q.transpose();
    }

    void operator<<(const Matrix<float, 3, 2>& rhs)
    {
        p = rhs.col(0).transpose();
        q = rhs.col(1).transpose();
    }

    MagnetometerData correct(const MagnetometerData& input) const override
    {
        MagnetometerData output;
        Vector3f vec;

        input >> vec;
        vec = vec.cwiseProduct(p) + q;
        output << vec;

        return output;
    }

private:
    Vector3f p, q;
};

class SoftIronCalibration
    : public AbstractCalibrationModel<MagnetometerData, MagnetometerData>
{
public:
    SoftIronCalibration(unsigned _maxSamples)
        : samples(_maxSamples, 7), numSamples(0),
          maxSamples(_maxSamples)
    {
    }

    bool feed(const MagnetometerData& data) override
    {
        if (numSamples >= maxSamples)
            return false;

        Vector3f vec;
        data >> vec;

        for(int i = 0; i < 3; ++i){
            samples(numSamples, i) = vec[i]*vec[i];
            samples(numSamples, i+3) = vec[i];
        }
        samples(numSamples, 6) = 1;

        numSamples++;
        return true;
    }

    ValuesCorrector<MagnetometerData>* computeResult() override
    {
        // TODO!
        return NULL;
    }

private:
    /*
     * The matrix contains x, y, z, x^2, y^2, z^2 and a column of 1s
     * row. Its shape is (N x 7)
     */
    MatrixXf samples;
    Vector3f ref;
    unsigned numSamples, maxSamples;
};


