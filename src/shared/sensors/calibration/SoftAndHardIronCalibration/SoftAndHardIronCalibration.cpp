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
#include <numeric>

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

    // keep the raw sample around for computeResultSym() because it needs the
    // whole dataset to normalize     it
    samples.push_back(vector);

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
    {
        if (minValue > eigenValues[i])
        {
            minValue = eigenValues[i];
            minIdx   = i;
        }
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

TwelveParametersCorrector SoftAndHardIronCalibration::computeResultSym()
{
    const int n = samples.size();

    if (n < 10)
    {
        lastSymFitWasValidEllipsoid = false;
        return TwelveParametersCorrector(Matrix3f::Identity(),
                                         Vector3f::Zero());
    }

    /*              ----
        Normalization (@ 19-24 symmag)
                    ----                */
    Vector3f offset =
        std::accumulate(samples.begin(), samples.end(), Vector3f::Zero()) /
        n;  // -> mean(data)

    Vector3f variance = Vector3f::Zero();
    for (const auto& s : samples)
    {
        Vector3f diff = s - offset;
        variance += diff.cwiseProduct(diff);
    }  // sum of (x-mean)^2

    Vector3f std = (variance / (n - 1)).cwiseSqrt();  // = std(data)
    float scale  = std.maxCoeff();                    //= max(std(data))
    if (scale <= 0)
        scale = 1.0f;

    /*              ----
     building D matrix (@ 26- 38 symmag)
                    ----                */

    Matrix<float, 10, 10> Dsym =
        Matrix<float, 10, 10>::Zero();  // Ax^2 + Bxy + Cxz + Dy^2 + Eyz + Fz^2
                                        // + Gx + Hy + Iz + J = 0
    for (const auto& s : samples)
    {
        Vector3f xn = (s - offset) / scale;
        Matrix<float, 10, 1> S;
        S << xn.x() * xn.x(), 2 * xn.x() * xn.y(), 2 * xn.x() * xn.z(),
            xn.y() * xn.y(), 2 * xn.y() * xn.z(), xn.z() * xn.z(), xn.x(),
            xn.y(), xn.z(), 1;

        Dsym += S * S.transpose();
    }

    // selfAjonitEigen should organize the autovectors having the smallest at
    // [0]
    SelfAdjointEigenSolver<Matrix<float, 10, 10>> symSolver(Dsym);
    Matrix<float, 10, 1> solx = symSolver.eigenvectors().col(0);

    /*              ----
      building R matrix (@ 48-59 symmag)
                     ----                */
    //  R : coeff of [x^2, xy, xz, y^2 yz, z^2] in a symmetric 3x3 matrix
    Matrix3f R;
    R(0, 0) = solx(0);
    R(0, 1) = R(1, 0) = solx(1);
    R(0, 2) = R(2, 0) = solx(2);
    R(1, 1)           = solx(3);
    R(1, 2) = R(2, 1) = solx(4);
    R(2, 2)           = solx(5);

    float dR = R.determinant();

    lastSymFitWasValidEllipsoid = true;
    {
        SelfAdjointEigenSolver<Matrix3f> esR(R, EigenvaluesOnly);
        Vector3f eigR = esR.eigenvalues();

        if ((eigR.array() < 0).all())
        {
            // opposite-sign ellipsoid: flip everything
            R    = -R;
            solx = -solx;
            dR   = -dR;
        }
        else if (!(eigR.array() > 0).all())
        {
            // mixed-sign eigenvalues: the fitted quadric is not an ellipsoid :
            // result is returned anyway but the caller should check
            // isLastSymFitValidEllipsoid() before trusiting it
            lastSymFitWasValidEllipsoid = false;
        }
    }

    /*  hard iron offset (@ 60, symmamg) */
    Vector3f linear = solx.segment<3>(6);
    Vector3f b      = -0.5f * R.inverse() * linear;

    /*              ---
        Soft iron matrix (@ 73-75, symmag)
                    ---                     */
    // sqrtm from matlab doesnt have an equivalent in Eigen :)
    // so: autovectors -> autovalues -> rebuild
    Matrix3f Rnew = R / std::cbrt(dR);

    SelfAdjointEigenSolver<Matrix3f> esRnew(Rnew);
    Vector3f sqrtEigenvalues = esRnew.eigenvalues().cwiseMax(0.0f).cwiseSqrt();
    Matrix3f A = esRnew.eigenvectors() * sqrtEigenvalues.asDiagonal() *
                 esRnew.eigenvectors().transpose();

    /* back to scale (b = offset' + scale*b; @ symmag) */
    Vector3f bTrue = offset + scale * b;

    return TwelveParametersCorrector(
        A,
        -A * bTrue);  // TwelveParametersCorrector::correct(x) = W * x + V, but
                      // we want corrected = A * (x - bTrue) = A * x - A * bTrue
}

}  // namespace Boardcore
