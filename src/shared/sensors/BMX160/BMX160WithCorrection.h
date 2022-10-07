/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Riccardo Musso, Luca Conterio, Alberto Nidasio
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

#include <diagnostic/PrintLogger.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/calibration/AxisOrientation.h>
#include <sensors/calibration/BiasCalibration/BiasCalibration.h>
#include <sensors/correction/SixParametersCorrector/SixParametersCorrector.h>
#include <utils/Stats/Stats.h>

#include "BMX160WithCorrectionData.h"

namespace Boardcore
{

/**
 * @brief Applies calibration to a BMX160
 *
 * Gets samples from a BMX160 and applies a specified correction and rotation.
 * It also calibrates the gyroscope.
 */
class BMX160WithCorrection : public Sensor<BMX160WithCorrectionData>
{
public:
    /**
     * @param bmx160 Already initialized bmx.
     * @param rotation Axis rotation.
     */
    explicit BMX160WithCorrection(BMX160* bmx160,
                                  AxisOrthoOrientation rotation = {
                                      Direction::POSITIVE_X,
                                      Direction::POSITIVE_Y});

    bool init() override;

    bool selfTest() override;

    void startCalibration();

    void stopCalibration();

private:
    BMX160WithCorrectionData sampleImpl() override;

    /**
     * @brief Rotates data axes as specified.
     */
    BMX160WithCorrectionData rotateAxis(BMX160WithCorrectionData data);

    BMX160* bmx160;

    AxisOrthoOrientation rotation;

    SixParametersCorrector accelerometerCorrector;
    SixParametersCorrector magnetometerCorrector;
    Eigen::Vector3f gyroscopeBias;

    bool calibrating      = false;
    int calibrationPoints = 0;

    PrintLogger logger = Logging::getLogger("bmx160wc");
};

}  // namespace Boardcore
