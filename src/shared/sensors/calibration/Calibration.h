/* Copyright (c) 2021-2022 Skyward Experimental Rocketry
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
 * @brief This class is used to adjust the values given by a sensor.
 *
 * @tparam T Type of sensor data the model is applied to (e.g.
 * GyroscopeData, MagnetometerData).
 */
template <typename T>
class ValuesCorrector
{
public:
    /**
     * @brief Sets the internal coefficients so that the corrected
     * values are exactly the same of the inputted ones.
     */
    virtual void setIdentity() = 0;

    /**
     * @brief Applies the correction to the given input.
     */
    virtual T correct(const T& input) const = 0;
};

/**
 * @brief Utility to compute calibration parameters for a specific sensor.
 *
 * The calibration appens in two phases:
 * - Sample collection: feed is called to store values;
 * - Coefficient computation: computeResult is used to calculate the correction
 * parameters.
 *
 * @tparam T Sensor's data type used in the feed function.
 * @tparam C ValuesCorrector returned by computeResult.
 */
template <typename T, typename C, typename... AdditionalFeedParams>
class AbstractCalibration
{
public:
    /**
     * @brief Stores the measurement for further processing.
     *
     * @returns false if the model can't accept the given data (usually because
     * the internal buffers are full).
     */
    virtual bool feed(const T& measurement,
                      const AdditionalFeedParams&... params) = 0;

    /**
     * @brief Uses the recorded measurements to computer the correction
     * parameters needed to correct sensor's data.
     *
     * Note: You need to feed the algorithm enough samples. Otherwise the method
     * returns an ineffective corrector.
     *
     * @return ValuesCorrector containing the correction parameters.
     */
    virtual C computeResult() = 0;
};

}  // namespace Boardcore
