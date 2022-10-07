/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#pragma once

#include <sensors/correction/BiasCorrector/BiasCorrector.h>

#include <Eigen/Core>
#include <string>

namespace Boardcore
{

/**
 * @brief Six-parameter correction uses, for each axis, a coefficient to be
 * multiplied and a constant to be added, so that is verified the formula:
 *
 * |x'|   ┌ |x|   |b0| ┐
 * |y'| = | |y| - |b1| | * |A1 A2 A3|
 * |z'|   └ |z|   |b2| ┘
 */
class SixParametersCorrector : public BiasCorrector
{
public:
    SixParametersCorrector();
    SixParametersCorrector(Eigen::Vector3f A, Eigen::Vector3f b);

    bool fromFile(const std::string& fileName) override;
    bool toFile(const std::string& fileName) override;

    Eigen::Vector3f correct(const Eigen::Vector3f& input) const override;

    Eigen::Vector3f getA() const;
    void setA(const Eigen::Vector3f& A);

protected:
    Eigen::Vector3f A;
};

}  // namespace Boardcore
