/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Federico Lolli
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

// ======= Sensor Fault Detection Model (SFDDescent) =======

#pragma once

#include <models/SVM.h>

#include <Eigen/Core>

#include "SFDCommon.h"

namespace Boardcore
{

class SFDDescent
{
public:
    static constexpr int NUM_FEATURES = 5;

    using SVMn        = SVM<NUM_FEATURES>;
    using FeaturesVec = Eigen::Vector<float, NUM_FEATURES>;

    struct SFDDescentConfig
    {
        SVMn::SVMConfig modelParameters;
    };

    explicit SFDDescent(const SFDDescentConfig& config);

    bool classify(const SFDVectorIn& input);

private:
    SVMn svm;

    FeaturesVec getFeatures(const SFDVectorIn& input);
};

}  // namespace Boardcore
