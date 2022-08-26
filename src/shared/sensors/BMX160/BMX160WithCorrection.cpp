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

#include "BMX160WithCorrection.h"

#include <miosix.h>

#include <fstream>

using namespace miosix;
using namespace Eigen;

namespace Boardcore
{

BMX160CorrectionParameters::BMX160CorrectionParameters()
{
    accelParams << 1, 0, 1, 0, 1, 0;    // cppcheck-suppress constStatement
    magnetoParams << 0, 0, 0, 0, 0, 0;  // cppcheck-suppress constStatement
}

std::string BMX160CorrectionParameters::header()
{
    return "accAx,accAy,accAz,accbx,accby,accbz,"
           "magAx,magAy,magAz,magbx,magby,magbz";
}

void BMX160CorrectionParameters::read(std::istream& inputStream)
{
    // Read accelerometer parameters
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            inputStream >> accelParams(j, i);
            inputStream.ignore(1, ',');
        }
    }

    // Read magnetometer parameters
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            inputStream >> magnetoParams(j, i);
            inputStream.ignore(1, ',');
        }
    }
}

void BMX160CorrectionParameters::print(std::ostream& outputStream) const
{
    // Print accelerometer data
    outputStream << accelParams(0, 0) << "," << accelParams(1, 0) << ","
                 << accelParams(2, 0) << "," << accelParams(0, 1) << ","
                 << accelParams(1, 1) << "," << accelParams(2, 1) << ",";

    // Print magnetometer  data
    outputStream << magnetoParams(0, 0) << "," << magnetoParams(1, 0) << ","
                 << magnetoParams(2, 0) << "," << magnetoParams(0, 1) << ","
                 << magnetoParams(1, 1) << "," << magnetoParams(2, 1) << ",";
}

BMX160WithCorrection::BMX160WithCorrection(
    BMX160* bmx160, BMX160CorrectionParameters correctionParameters,
    AxisOrthoOrientation rotation)
    : bmx160(bmx160), rotation(rotation)
{
    accelerometerCorrector << correctionParameters.accelParams;
    magnetometerCorrector << correctionParameters.magnetoParams;
}

BMX160WithCorrection::BMX160WithCorrection(
    BMX160* bmx160, BMX160CorrectionParameters correctionParameters)
    : bmx160(bmx160)
{
    accelerometerCorrector << correctionParameters.accelParams;
    magnetometerCorrector << correctionParameters.magnetoParams;
}

BMX160WithCorrection::BMX160WithCorrection(BMX160* bmx160) : bmx160(bmx160) {}

bool BMX160WithCorrection::init() { return true; }

bool BMX160WithCorrection::selfTest() { return true; }

void BMX160WithCorrection::startCalibration()
{
    gyroscopeCalibrator.reset();
    calibrating = true;
}

void BMX160WithCorrection::stopCalibration()
{
    calibrating = false;

    {
        PauseKernelLock lock;
        gyroscopeCorrector = gyroscopeCalibrator.computeResult();
    }

    // Print the calibrator data
    Vector3f gyroscopeCorrectionParameters;
    gyroscopeCorrector >> gyroscopeCorrectionParameters;
    LOG_INFO(logger, "Gyroscope bias vector from calibration\n");
    LOG_INFO(logger, "b = [    {: >2.5f}    {: >2.5f}    {: >2.5f}    ]\n\n",
             gyroscopeCorrectionParameters(0), gyroscopeCorrectionParameters(1),
             gyroscopeCorrectionParameters(2));
}

BMX160CorrectionParameters
BMX160WithCorrection::readCorrectionParametersFromFile(const char* fileName)
{
    BMX160CorrectionParameters correctionParameters;
    std::ifstream input(fileName);

    // Ignore header line (csv header)
    input.ignore(1000, '\n');

    correctionParameters.read(input);

    return correctionParameters;
}

BMX160WithCorrectionData BMX160WithCorrection::sampleImpl()
{
    if (!bmx160)
    {
        LOG_ERR(logger, "Driver doesn't point to valid sensor");
        return BMX160WithCorrectionData{};
    }

    Eigen::Vector3f avgAccel{0, 0, 0}, avgMag{0, 0, 0}, avgGyro{0, 0, 0}, vec;
    uint64_t accelTimestamp = 0, magTimestamp = 0, gyroTimestamp = 0;
    uint8_t fifoSize, numAccel = 0, numMag = 0, numGyro = 0;
    BMX160Data fifoElement;
    BMX160WithCorrectionData result;

    fifoSize = bmx160->getLastFifoSize();

    // Read all data in the fifo
    for (int i = 0; i < fifoSize; i++)
    {
        fifoElement = bmx160->getFifoElement(i);

        // Read acceleration data
        if (fifoElement.accelerationTimestamp > accelTimestamp)
        {
            static_cast<AccelerometerData>(fifoElement) >> vec;
            avgAccel += vec;

            accelTimestamp = fifoElement.accelerationTimestamp;
            numAccel++;
        }

        // Read magnetometer data
        if (fifoElement.magneticFieldTimestamp > magTimestamp)
        {
            static_cast<MagnetometerData>(fifoElement) >> vec;
            avgMag += vec;

            magTimestamp = fifoElement.magneticFieldTimestamp;
            numMag++;
        }

        // Read gyroscope data
        if (fifoElement.angularVelocityTimestamp > gyroTimestamp)
        {
            static_cast<GyroscopeData>(fifoElement) >> vec;
            avgGyro += vec;

            gyroTimestamp = fifoElement.angularVelocityTimestamp;
            numGyro++;
        }
    }

    // Average the samples
    if (numAccel == 0)
        static_cast<AccelerometerData>(bmx160->getLastSample()) >> avgAccel;
    else
        avgAccel /= numAccel;

    if (numMag == 0)
        static_cast<MagnetometerData>(bmx160->getLastSample()) >> avgMag;
    else
        avgMag /= numMag;

    if (numGyro == 0)
        static_cast<GyroscopeData>(bmx160->getLastSample()) >> avgGyro;
    else
        avgGyro /= numGyro;

    static_cast<AccelerometerData&>(result) << avgAccel;
    static_cast<MagnetometerData&>(result) << avgMag;
    static_cast<GyroscopeData&>(result) << avgGyro;

    // Correct the averaged measurements
    result = accelerometerCorrector.correct(result);
    result = magnetometerCorrector.correct(result);
    result = gyroscopeCorrector.correct(result);

    // Get the timestamp of the newest value in fifo
    result.accelerationTimestamp    = fifoElement.accelerationTimestamp;
    result.magneticFieldTimestamp   = fifoElement.accelerationTimestamp;
    result.angularVelocityTimestamp = fifoElement.accelerationTimestamp;

    if (calibrating)
        gyroscopeCalibrator.feed(result);

    return rotateAxis(result);
}

BMX160WithCorrectionData BMX160WithCorrection::rotateAxis(
    BMX160WithCorrectionData data)
{
    // Rotate the axes according to the given rotation

    // Accelerometer
    AccelerometerData accData = data;
    Eigen::Vector3f accDataVector;
    accData >> accDataVector;
    accDataVector = rotation.getMatrix() * accDataVector;
    accData << accDataVector;

    // Gyroscope
    GyroscopeData gyrData = data;
    Eigen::Vector3f gyrDataVector;
    gyrData >> gyrDataVector;
    gyrDataVector = rotation.getMatrix() * gyrDataVector;
    gyrData << gyrDataVector;

    // Magnetometer
    MagnetometerData magData = data;
    Eigen::Vector3f magDataVector;
    magData >> magDataVector;
    magDataVector = rotation.getMatrix() * magDataVector;
    magData << magDataVector;

    data = accData;
    data = gyrData;
    data = magData;

    return data;
}

}  // namespace Boardcore
