/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Riccardo Musso
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

/*  Please choose one between accelerometer or magnetometer */
#define TEST_ACCELEROMETER_DATA 1
#define TEST_MAGNETOMETER_DATA 0

/*  ACCELEROMETER calibration: please set the chosen one to 1  */
#define BIAS_CALIBRATION_TEST 0
#define SIX_PARAMETER_CALIBRATION_TEST 0
#define TWELVE_PARAMETER_CALIBRATION_TEST 1

/*  MAGNETOMETER calibration: please set the chosen one to 1 */
#define HARD_IRON_CALIBRATION_TEST 1
#define SOFT_IRON_CALIBRATION_TEST 0

#include <Common.h>
#include <drivers/spi/SPIDriver.h>
#include <miosix.h>
#include <sensors/LIS3DSH/LIS3DSH.h>
#include <sensors/calibration/BiasCalibration.h>
#include <sensors/calibration/HardIronCalibration.h>
#include <sensors/calibration/SixParameterCalibration.h>
#include <sensors/calibration/SoftIronCalibration.h>
#include <sensors/calibration/TwelveParameterCalibration.h>

#if TEST_ACCELEROMETER_DATA
#include "accelerometer-data.h"
#elif TEST_MAGNETOMETER_DATA
#include "magnetometer-data.h"
#endif

using namespace Boardcore;
using namespace miosix;
using namespace Eigen;

int main()
{
#if TEST_ACCELEROMETER_DATA

    constexpr unsigned NumSamples = accData::nOrientations * accData::nSamples;

#if BIAS_CALIBRATION_TEST
    auto* model = new BiasCalibration<AccelerometerData>;

    TRACE("Using BIAS calibration model.\n");
#endif

#if SIX_PARAMETER_CALIBRATION_TEST
    auto* model = new SixParameterCalibration<AccelerometerData, NumSamples>;
    model->setReferenceVector({0, 0, 1});

    TRACE("Using SIX-PARAMETER calibration model.\n");
#endif

#if TWELVE_PARAMETER_CALIBRATION_TEST
    auto* model = new TwelveParameterCalibration<AccelerometerData, NumSamples>;
    model->setReferenceVector({0, 0, 1});

    TRACE("Using TWELVE-PARAMETER calibration model.\n");
#endif

    TRACE("Feeding accelerometer data to the calibration model ... \n");

    for (unsigned i = 0; i < accData::nOrientations; i++)
        for (unsigned j = 0; j < accData::nSamples; j++)
            model->feed(accData::samples[i][j], accData::orientations[i]);

    TRACE("Computing the result ... \n");

    auto corrector = model->computeResult();

    TRACE("Now testing with the same data: \n");

    float err0 = 0.f, err1 = 0.f;
    Vector3f vec0 = {0, 0, 0}, vec1 = {0, 0, 0};

    for (unsigned i = 0; i < accData::nOrientations; i++)
    {
        TRACE(
            "  --- Orientation n. %d: X axis oriented toward %s, Y axis "
            "oriented toward %s\n",
            i,
            humanFriendlyDirection[static_cast<uint8_t>(
                accData::orientations[i].xAxis)],
            humanFriendlyDirection[static_cast<uint8_t>(
                accData::orientations[i].yAxis)]);

        for (unsigned j = 0; j < accData::nSamples; j++)
        {
            AccelerometerData out;
            Vector3f exact, before, after, d0, d1;

            exact = accData::orientations[i].getMatrix().transpose() *
                    Vector3f{0, 0, 1};
            out = corrector.correct(accData::samples[i][j]);
            out >> after;
            accData::samples[i][j] >> before;

            /* Every five samples, print one */
            if (j % 5 == 0)
            {
                TRACE("%f %f %f -> %f %f %f\n", before[0], before[1], before[2],
                      after[0], after[1], after[2]);
            }

            d0 = exact - before;
            d1 = exact - after;

            err0 += d0.norm();
            err1 += d1.norm();

            vec0 += Vector3f{abs(d0[0]), abs(d0[1]), abs(d0[2])};
            vec1 += Vector3f{abs(d1[0]), abs(d1[1]), abs(d1[2])};
        }
    }

    const unsigned n = accData::nOrientations * accData::nSamples;

    err0 /= n;
    err1 /= n;
    vec0 /= n;
    vec1 /= n;

    TRACE(
        "Average norm of displacement vector on accelerometer data: %.3f "
        "(before it was %.3f)\n",
        err1, err0);
    TRACE("Here is the average error per axis (in percentage):\n");
    TRACE("Before calibration:\t\tx: %3.3f%%\ty: %3.3f%%\tz: %3.3f%%\n",
          vec0[0] * 100, vec0[1] * 100, vec0[2] * 100);
    TRACE("After calibration:\t\tx: %3.3f%%\ty: %3.3f%%\tz: %3.3f%%\n",
          vec1[0] * 100, vec1[1] * 100, vec1[2] * 100);

    TRACE("The parameters are:\n\n");

#if BIAS_CALIBRATION_TEST
    Vector3f bias;
    corrector >> bias;

    TRACE("b: the bias vector\n");
    TRACE("b = [    % 2.5f    % 2.5f    % 2.5f    ]\n\n", bias[0], bias[1],
          bias[2]);
#endif

#if SIX_PARAMETER_CALIBRATION_TEST
    Matrix<float, 3, 2> m;
    corrector >> m;

    TRACE("b: the bias vector\n");
    TRACE("b = [    % 2.5f    % 2.5f    % 2.5f    ]\n\n", m(0, 1), m(1, 1),
          m(2, 1));

    TRACE("M: the matrix to be multiplied to the input vector\n");

    TRACE("    |    % 2.5f    % 2.5f    % 2.5f    |\n", m(0, 0), 0.f, 0.f);
    TRACE("M = |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f, m(1, 0), 0.f);
    TRACE("    |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f, 0.f, m(2, 0));
#endif

#if TWELVE_PARAMETER_CALIBRATION_TEST
    Matrix<float, 3, 4> m;
    corrector >> m;

    TRACE("b: the bias vector\n");
    TRACE("b = [    % 2.5f    % 2.5f    % 2.5f    ]\n\n", m(0, 1), m(1, 1),
          m(2, 1));

    TRACE("M: the matrix to be multiplied to the input vector\n");

    TRACE("    |    % 2.5f    % 2.5f    % 2.5f    |\n", m(0, 0), m(0, 1),
          m(0, 2));
    TRACE("M = |    % 2.5f    % 2.5f    % 2.5f    |\n", m(1, 0), m(1, 1),
          m(1, 2));
    TRACE("    |    % 2.5f    % 2.5f    % 2.5f    |\n", m(2, 0), m(2, 1),
          m(2, 2));
#endif

#endif /* #if TEST_ACCELEROMETER_DATA */

#if TEST_MAGNETOMETER_DATA

    constexpr unsigned NumSamples = magnetoData::nSamples;

#if HARD_IRON_CALIBRATION_TEST
    auto* model = new HardIronCalibration<NumSamples>;

    TRACE("Using Hard-iron correction model.\n");
#endif

#if SOFT_IRON_CALIBRATION_TEST
    auto* model = new SoftIronCalibration<NumSamples>;

    TRACE("Using Soft-iron correction model.\n");
#endif

    TRACE("Feeding magnetometer data to the calibration model ... \n");

    for (unsigned i = 0; i < magnetoData::nSamples; i++)
        model->feed(magnetoData::samples[i]);

    TRACE("Computing the result ... \n");

    auto corrector = model->computeResult();

    TRACE("Done.\n");

    /* Prints only 1/10 of samples */
    for (unsigned i = 0; i < magnetoData::nSamples; i += 10)
    {
        MagnetometerData out;
        Vector3f before, after;

        out = corrector.correct(magnetoData::samples[i]);
        out >> after;
        magnetoData::samples[i] >> before;

        TRACE("%f %f %f -> %f %f %f\n", before[0], before[1], before[2],
              after[0], after[1], after[2]);
    }

    TRACE("The parameters are:\n\n");

#if HARD_IRON_CALIBRATION_TEST
    Vector3f bias;
    corrector >> bias;

    TRACE("v: the hard iron correction\n");
    TRACE("v = [    % 2.5f    % 2.5f    % 2.5f    ]\n\n", bias[0], bias[1],
          bias[2]);
#endif

#if SOFT_IRON_CALIBRATION_TEST
    Matrix<float, 3, 2> m;
    corrector >> m;

    TRACE("v: the hard iron correction\n");
    TRACE("v = [    % 2.5f    % 2.5f    % 2.5f    ]\n\n", m(0, 1), m(1, 1),
          m(2, 1));

    TRACE("W: the soft iron correction\n");

    TRACE("    |    % 2.5f    % 2.5f    % 2.5f    |\n", m(0, 0), 0.f, 0.f);
    TRACE("W = |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f, m(1, 0), 0.f);
    TRACE("    |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f, 0.f, m(2, 0));
#endif

#endif /* #if TEST_MAGNETOMETER_DATA */
}
