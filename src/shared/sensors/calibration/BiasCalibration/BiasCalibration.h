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

#include <sensors/SensorData.h>
#include <sensors/calibration/AxisOrientation.h>
#include <sensors/calibration/SensorDataExtra/SensorDataExtra.h>
#include <sensors/correction/BiasCorrector/BiasCorrector.h>

#include <Eigen/Core>

namespace Boardcore
{

/**
 * @brief This is the dumbest type of calibration possible: an offset.
 *
 * During the calibration phase it will use a given reference vector (for
 * example the gravitational acceleration), and every time you'll feed the model
 * with a new value, you have to give it the orientation of the sensor, so it
 * can guess the bias.
 */
class BiasCalibration
{
public:
    BiasCalibration();
    explicit BiasCalibration(Eigen::Vector3f ref);

    void setReferenceVector(Eigen::Vector3f ref);
    Eigen::Vector3f getReferenceVector();

    void reset();

    void feed(const Eigen::Vector3f& measured,
              const AxisOrientation& transform);
    void feed(const Eigen::Vector3f& measured);

    BiasCorrector computeResult();

private:
    Eigen::Vector3f mean = {0, 0, 0};
    Eigen::Vector3f ref;
    unsigned numSamples = 0;
};

}  // namespace Boardcore
