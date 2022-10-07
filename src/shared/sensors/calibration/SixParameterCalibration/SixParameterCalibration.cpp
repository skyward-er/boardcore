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

#include "SixParameterCalibration.h"

using namespace Eigen;

namespace Boardcore
{

SixParameterCalibration::SixParameterCalibration() {}

SixParameterCalibration::SixParameterCalibration(Vector3f ref) : ref(ref) {}

void SixParameterCalibration::setReferenceVector(Vector3f ref)
{
    this->ref = ref;
}

Vector3f SixParameterCalibration::getReferenceVector() { return ref; }

void SixParameterCalibration::feed(const Eigen::Vector3f& measured,
                                   const AxisOrientation& transform)
{
    // Add a new row by resizing the matrix
    samples.conservativeResize(numSamples + 1, 6);

    samples.block(numSamples, 0, 1, 3) = measured.transpose();
    samples.block(numSamples, 3, 1, 3) =
        (transform.getMatrix() * ref).transpose();

    numSamples++;
}

SixParametersCorrector SixParameterCalibration::computeResult()
{
    Vector3f A, b;

    for (int i = 0; i < 3; ++i)
    {
        MatrixXf coeffs(numSamples, 2);

        Eigen::VectorXf terms = samples.block(0, i + 3, numSamples, 1);

        Vector2f solution;
        coeffs.fill(1);
        coeffs.block(0, 0, numSamples, 1) = samples.block(0, i, numSamples, 1);

        auto solver = coeffs.colPivHouseholderQr();
        solution    = solver.solve(terms);

        A(i) = solution(0);
        b(i) = solution(1);
    }

    return {A, b};
}

}  // namespace Boardcore
