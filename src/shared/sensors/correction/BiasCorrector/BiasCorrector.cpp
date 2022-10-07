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

#include "BiasCorrector.h"

#include <fstream>

using namespace Eigen;

namespace Boardcore
{

BiasCorrector::BiasCorrector() : b(0, 0, 0) {}

BiasCorrector::BiasCorrector(const Eigen::Vector3f& b) : b(b) {}

bool BiasCorrector::fromFile(const std::string& fileName)
{
    std::ifstream input(fileName);

    if (input)
    {
        // Ignore header line (csv header)
        input.ignore(1000, '\n');

        for (int i = 0; i < 3; i++)
        {
            input >> b(i);
            input.ignore(1, ',');
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool BiasCorrector::toFile(const std::string& fileName)
{
    std::ofstream output(fileName);

    if (output)
    {
        output << "b0,b1,b2" << std::endl;
        output << b(0) << "," << b(1) << "," << b(1);

        return true;
    }
    else
    {
        return false;
    }
}

Eigen::Vector3f BiasCorrector::getb() const { return b; }

void BiasCorrector::setb(const Eigen::Vector3f& b) { this->b = b; }

Vector3f BiasCorrector::correct(const Vector3f& data) const { return data - b; }

}  // namespace Boardcore
