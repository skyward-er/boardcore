/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Tommaso Lamon
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
#include <string>

namespace Boardcore
{
/**
 * @brief Twelve parameter correction works by multiplying the real measurement
 * of a sensor by the W matrix (including the off-diagonal terms) and by adding
 * to the result of the product the constant vector V
 *
 * a_b,ideal = (W * a_b,real) + V =
 *
 *             | w11 w21 w31 |              | Vx |
 *          =  | w12 w22 w32 | * a_b,real + | Vy |
 *             | w13 w23 w33 |              | Vz |
 *
 */
class TwelveParametersCorrector
{
public:
    TwelveParametersCorrector(Eigen::Matrix3f W, Eigen::Vector3f V);
    TwelveParametersCorrector();

    /**
     * @brief Reads the .csv file for the coefficients to configure the W and V
     * terms of the system
     * @param filename The name of the file
     * @returns A boolean value to signal the outcome of the reading
     */
    bool fromFile(const std::string& filename);

    /**
     * @brief Writes the .csv file with the coefficients of W and V
     * @param filename The name of the file
     * @returns A boolean value to signal the outcome of the writing
     */
    bool toFile(const std::string& filename);

    /**
     * @brief Applies the correction to a_b,real
     * @param inputVector The vector a_b,real
     * @returns The a_b,ideal vector
     */
    Eigen::Vector3f correct(const Eigen::Vector3f& inputVector) const;

    Eigen::Vector3f getV() const;
    void setV(const Eigen::Vector3f& V);

    Eigen::Matrix3f getW() const;
    void setW(const Eigen::Matrix3f& W);

protected:
    Eigen::Vector3f V;
    Eigen::Matrix3f W;
};

}  // namespace Boardcore
