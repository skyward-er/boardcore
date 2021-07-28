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

#include <sensors/BMX160/BMX160.h>
#include <sensors/calibration/BiasCalibration.h>
#include <sensors/calibration/SixParameterCalibration.h>

#include "BMX160WithCorrectionData.h"

struct BMX160CorrectionParameters
{
    Matrix<float, 3, 2> accelParams, magnetoParams;
    int minGyroSamplesForCalibration = 0;

    BMX160CorrectionParameters()
    {
        accelParams << 0, 0, 0, 0, 0, 0;
        magnetoParams << 0, 0, 0, 0, 0, 0;
    }

    static std::string header()
    {
        return "accel_p1,accel_p2,accel_p3,accel_q1,accel_q2,accel_q3,mag_p1,"
               "mag_p2,mag_p3,mag_q1,mag_q2,mag_q3,"
               "minGyroSamplesForCalibration";
    }

    void read(std::istream& inputStream);

    void print(std::ostream& outputStream) const;
};

/**
 * @brief Driver for BMX160 with calibration.
 *
 * TODO
 */
class BMX160WithCorrection : public Sensor<BMX160WithCorrectionData>
{
public:
    // TODO: Add a constructon with the rotation matrix

    /**
     * @param correctionParameters correction parameter to apply.
     * @param driver_ already initialized bmx.
     */
    BMX160WithCorrection(BMX160* bmx160_,
                         BMX160CorrectionParameters correctionParameters);

    /**
     * @brief Constructor without correction parameters, no correciton will be
     * applied.
     *
     * @param bmx160_ correction parameter to apply.
     */
    BMX160WithCorrection(BMX160* bmx160_);

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

private:
    BMX160WithCorrectionData sampleImpl() override;

    /**
     * @brief Rotates data axes as specified by the rotation matrix.
     */
    BMX160WithCorrectionData rotateAxis(BMX160WithCorrectionData data);

    BMX160* bmx160;

    int minGyroSamplesForCalibration = 200;

    SixParameterCorrector<AccelerometerData> accelerometerCorrector;
    SixParameterCorrector<MagnetometerData> magnetometerCorrector;
    BiasCorrector<GyroscopeData> gyroscopeCorrector{};
};