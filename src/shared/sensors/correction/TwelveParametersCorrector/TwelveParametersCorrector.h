/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Tommaso Lamon
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

class TwelveParametersCorrector : public BiasCorrector
{
    TwelveParametersCorrector(Eigen::Matrix3f W, Eigen::Vector3f V);
    TwelveParametersCorrector();

    bool fromFile(const std::string& filename) override;
    bool toFile(const std::string& filename) override;

    Eigen::Vector3f correct(const Eigen::Vector3f& inputVector) const override;

    Eigen::Vector3f getV() const;
    void setV(const Eigen::Vector3f& V);

    Eigen::Matrix3f getW() const;
    void setW(const Eigen::Matrix3f& W);

protected:
    Eigen::Vector3f V;
    Eigen::Matrix3f W;
};

}  // namespace Boardcore
