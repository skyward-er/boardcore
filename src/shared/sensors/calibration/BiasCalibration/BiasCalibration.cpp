/* Copyright (c) 2020-2022 Skyward Experimental Rocketry
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

#include "BiasCalibration.h"

using namespace Eigen;

namespace Boardcore
{

BiasCalibration::BiasCalibration() : ref(0, 0, 0) {}

BiasCalibration::BiasCalibration(Vector3f ref) : ref(ref) {}

void BiasCalibration::setReferenceVector(Vector3f ref) { this->ref = ref; }

Vector3f BiasCalibration::getReferenceVector() { return ref; }

void BiasCalibration::reset() { mean = {0, 0, 0}; }

void BiasCalibration::feed(const Vector3f& measured,
                           const AxisOrientation& transform)
{
    feed(transform.getMatrix() * ref);
}

void BiasCalibration::feed(const Vector3f& measured)
{
    numSamples++;
    mean += ((measured - ref) - mean) / numSamples;
}

BiasCorrector BiasCalibration::computeResult()
{
    if (numSamples == 0)
        return BiasCorrector{{0, 0, 0}};
    return BiasCorrector{mean};
}

}  // namespace Boardcore
