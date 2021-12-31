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
#include <sensors/calibration/BiasCalibration.h>
#include <sensors/calibration/SixParameterCalibration.h>

#include "BMX160WithCorrectionData.h"

namespace Boardcore
{

/**
 * Holds correction parameters for BMX160.
 */
struct BMX160CorrectionParameters
{
    Eigen::Matrix<float, 3, 2> accelParams, magnetoParams;
    int minGyroSamplesForCalibration = 0;

    BMX160CorrectionParameters();

    static std::string header();

    void read(std::istream& inputStream);

    void print(std::ostream& outputStream) const;
};

/**
 * @brief Driver for BMX160 with calibration.
 *
 * Gets samples from a BMX160 and applies a specified correction and rotation.
 * It also calibrates the gyroscope.
 */
class BMX160WithCorrection : public Sensor<BMX160WithCorrectionData>
{
public:
    /**
     * @param bmx160_ already initialized bmx.
     * @param correctionParameters correction parameter to apply.
     * @param rotation_ axis rotation.
     */
    BMX160WithCorrection(BMX160* bmx160_,
                         BMX160CorrectionParameters correctionParameters,
                         AxisOrthoOrientation rotation_);

    /**
     * Constructor without rotation, no rotation will be applied.
     *
     * @param bmx160_ already initialized bmx.
     * @param correctionParameters correction parameter to apply.
     */
    BMX160WithCorrection(BMX160* bmx160_,
                         BMX160CorrectionParameters correctionParameters);

    /**
     * @brief Constructor without correction nor rotation, no correciton and
     * rotation will be applied.
     *
     * @param bmx160_ correction parameter to apply.
     */
    explicit BMX160WithCorrection(BMX160* bmx160_);

    bool init() override;

    bool selfTest() override;

    /**
     * @brief Performs the gyroscope calibration.
     *
     * The gyroscope calibration consists in averaging some samples to measure
     * the bias.
     * This function is intended to run while another thread samples the bmx at
     * at least 10Hz.
     */
    bool calibrate();

    /**
     * @brief Utility function to read correction parameters from file.
     */
    static BMX160CorrectionParameters readCorrectionParametersFromFile(
        const char* fileName);

    /**
     * @return Gyroscope calibration biases.
     */
    BMX160GyroscopeCalibrationBiases getGyroscopeBiases();

private:
    BMX160WithCorrectionData sampleImpl() override;

    /**
     * @brief Rotates data axes as specified.
     */
    BMX160WithCorrectionData rotateAxis(BMX160WithCorrectionData data);

    BMX160* bmx160;

    int minGyroSamplesForCalibration = 200;

    AxisOrthoOrientation rotation = {Direction::POSITIVE_X,
                                     Direction::POSITIVE_Y};

    SixParameterCorrector<AccelerometerData> accelerometerCorrector;
    SixParameterCorrector<MagnetometerData> magnetometerCorrector;
    BiasCorrector<GyroscopeData> gyroscopeCorrector{};

    Eigen::Vector3f gyroscopeCorrectionParameters;

    PrintLogger logger = Logging::getLogger("bmx160withcorrection");
};

}  // namespace Boardcore
