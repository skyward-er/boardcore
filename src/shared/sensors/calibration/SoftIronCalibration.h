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

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Common.h>

#include "Calibration.h"
#include "sensors/SensorData.h"

/*
 * The Soft-iron calibration removes the measurement error given by both the Hard and Soft Iron distortion of the magnetic field.
 */
class SoftIronCorrector : public ValuesCorrector<MagnetometerData>
{
public:
    SoftIronCorrector() : SoftIronCorrector({ 1, 1, 1 }, { 0, 0, 0 }) {}

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

template <unsigned MaxSamples>
class SoftIronCalibration
    : public AbstractCalibrationModel<MagnetometerData, SoftIronCorrector>
{
public:
    SoftIronCalibration()
        : samples(), numSamples(0)
    {
    }

    bool feed(const MagnetometerData& data) override
    {
        if (numSamples >= MaxSamples)
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

    SoftIronCorrector computeResult() override
    {
        using Mx = Matrix<float, 7, 7>;
        using Vec7 = Matrix<float, 7, 1>;

        float minValue, det;
        int minIdx;

        MatrixXf tmp1, tmp2;
        Vector3f p, q;
        Vec7 vec;
        Mx mat;

        tmp1 = samples.block(0, 0, numSamples, 7);
        tmp2 = tmp1.transpose();

        // mat = tmp2 * tmp1;

        /*
         * tmp1: N x 7
         * tmp2: 7 x N
         *
         * Big matrices multiplication: if done with Eigen, the program crashes (bus fault),
         * so we'll do that with good old for's
        */
        for(unsigned i = 0; i < 7; ++i){
            for(unsigned j = 0; j < 7; ++j){
                mat(i, j) = 0;

                for(unsigned k = 0; k < numSamples; ++k){
                    mat(i, j) += tmp2(i, k) * tmp1(k, j);
                }
            }
        }

        SelfAdjointEigenSolver<Mx> solver(mat);
        auto eigenvalues = solver.eigenvalues();

        minValue = eigenvalues[0];
        minIdx = 0;

        for(int i = 1; i < eigenvalues.rows(); ++i){
            if(minValue > eigenvalues[i]){
                minValue = eigenvalues[i];
                minIdx = i;
            }
        }

        vec = solver.eigenvectors().col(minIdx);
        det = vec[0] * vec[1] * vec[2];

        if(det < 0){
            vec *= -1;
            det *= -1;
        }

        p = vec.block(0, 0, 3, 1) / cbrt(det);
        q = {
            vec[3] / vec[0] / 2,
            vec[4] / vec[1] / 2,
            vec[5] / vec[2] / 2
        };

        p[0] = sqrt(p[0]);
        p[1] = sqrt(p[1]);
        p[2] = sqrt(p[2]);

        q = q.cwiseProduct(p);

        return {p, q};
    }

private:
    /*
     * The matrix contains x, y, z, x^2, y^2, z^2 and a column of 1s
     * row. Its shape is (N x 7)
     */
    Matrix<float, MaxSamples, 7> samples;
    unsigned numSamples;
};




