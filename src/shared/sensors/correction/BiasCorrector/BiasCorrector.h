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

#pragma once

#include <Eigen/Core>

namespace Boardcore
{

/**
 * @brief Bias correction removes a bias from a measurement.
 *
 * |x'|   |x|   |b0|
 * |y'| = |y| - |b1|
 * |z'|   |z|   |b2|
 */
class BiasCorrector
{
public:
    BiasCorrector();
    explicit BiasCorrector(const Eigen::Vector3f& b);

    virtual bool fromFile(const std::string& fileName);
    virtual bool toFile(const std::string& fileName);

    Eigen::Vector3f getb() const;
    void setb(const Eigen::Vector3f& b);

    virtual Eigen::Vector3f correct(const Eigen::Vector3f& data) const;

protected:
    Eigen::Vector3f b;
};

}  // namespace Boardcore
