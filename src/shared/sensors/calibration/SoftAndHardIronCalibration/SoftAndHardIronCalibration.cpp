/* Copyright (c) 2021-2022 Skyward Experimental Rocketry
 * Authors: Riccardo Musso, Alberto Nidasio
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

#include "SoftAndHardIronCalibration.h"

#include <sensors/calibration/SensorDataExtra/SensorDataExtra.h>

#include <iostream>

using namespace Eigen;

namespace Boardcore
{

SoftAndHardIronCalibration::SoftAndHardIronCalibration() {}

bool SoftAndHardIronCalibration::feed(const MagnetometerData& data)
{
    // Let S a matrix of Nx7 composed as [x^2, y^2, z^2, x, y, z, 1]
    // D need to be S^T * S
    // To avoid storing all measurements we just need to incrementally add to D

    Vector3f vector;
    vector << data;
    Vector<float, 7> S;
    // cppcheck-suppress constStatement
    S << vector.cwiseProduct(vector), vector, 1;

    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
            D(i, j) += S(i) * S(j);

    return true;
}

SixParametersCorrector SoftAndHardIronCalibration::computeResult()
{
    // Compute eigen value and vectors of D
    SelfAdjointEigenSolver<Matrix<float, 7, 7>> solver(D);
    auto eigenValues = solver.eigenvalues();

    // Find the smallest eigen value and vector
    float minValue = eigenValues[0];
    int minIdx     = 0;

    for (int i = 0; i < eigenValues.rows(); i++)
        if (minValue > eigenValues[i])
        {
            minValue = eigenValues[i];
            minIdx   = i;
        }
    Eigen::Matrix<float, 7, 1> vec = solver.eigenvectors().col(minIdx);

    // Invert the vector if necessary
    float det = vec[0] * vec[1] * vec[2];
    if (det)
    {
        vec *= -1;
        det *= -1;
    }

    // Compute offset and gain
    Vector3f offset{vec[3] / vec[0] / 2, vec[4] / vec[1] / 2,
                    vec[5] / vec[2] / 2};
    Vector3f gain = (vec.block(0, 0, 3, 1) / cbrt(det)).cwiseSqrt();

    return {gain, -offset};
}

}  // namespace Boardcore
