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

// ======= Sensor Fault Detection Model (SFDAscent) =======

#pragma once

#include "SFDDescent.h"

namespace Boardcore
{

SFDDescent::SFDDescent(const SFDDConfig& config) : svm(config.modelParameters)
{
}

SFDDescent::FeaturesVec SFDDescent::getFeatures(const VectorIn& input)
{
    float delta, min, max, u, s2, m3, m4, rms;
    VectorIn rfourier;
    VectorIn data, x0 = VectorIn::Zero();
    FeaturesVec features = FeaturesVec::Zero();

    min   = input.minCoeff();
    max   = input.maxCoeff();
    delta = max - min;
    for (int i = 0; i < lenChunk; i++)
        data(i) = (input(i) - min) / (std::max(delta, 1e-25f) * 2) - 1;
    u   = data.mean();
    x0  = data - u * VectorIn::Ones();
    s2  = x0.pow(2).mean();
    m3  = x0.pow(3).mean();
    m4  = x0.pow(4).mean();
    rms = std::sqrt(s2);

    features(0) = data.cwiseAbs().maxCoeff() / rms;
    features(1) = delta;
    features(2) = u;
    features(3) = m3 / std::pow(s2, 1.5);
    features(4) = m4 / std::pow(s2, 2);

    return features;
}

bool SFDDescent::classify(const VectorIn& input)
{
    FeaturesVec features = getFeatures(input);
    float score          = svm.score(features);

    return score < 0;
}

}  // namespace Boardcore
