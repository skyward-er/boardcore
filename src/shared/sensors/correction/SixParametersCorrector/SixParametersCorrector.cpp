/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Riccardo Musso, Alberto Nidasio
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

#include "SixParametersCorrector.h"

#include <fstream>

using namespace Eigen;

namespace Boardcore
{

SixParametersCorrector::SixParametersCorrector() : BiasCorrector(), A(1, 1, 1)
{
}

SixParametersCorrector::SixParametersCorrector(Eigen::Vector3f A,
                                               Eigen::Vector3f b)
    : BiasCorrector(b), A(A)
{
}

bool SixParametersCorrector::fromFile(const std::string& fileName)
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

        for (int i = 0; i < 3; i++)
        {
            input >> A(i);
            input.ignore(1, ',');
        }

        input.close();

        return true;
    }
    else
    {
        return false;
    }
}

bool SixParametersCorrector::toFile(const std::string& fileName)
{
    std::ofstream output(fileName);

    if (output)
    {
        output << "b0,b1,b2,A0,A1,A2" << std::endl;
        output << b(0) << "," << b(1) << "," << b(2) << ",";
        output << A(0) << "," << A(1) << "," << A(2);

        output.close();

        return true;
    }
    else
    {
        return false;
    }
}

Vector3f SixParametersCorrector::correct(const Vector3f& input) const
{
    // From magcal in MATLab: out = (in - b) * A
    return A.cwiseProduct(input - b);
}

Eigen::Vector3f SixParametersCorrector::getA() const { return A; }

void SixParametersCorrector::setA(const Eigen::Vector3f& A) { this->A = A; }

}  // namespace Boardcore
