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

BMX160WithCorrection::BMX160WithCorrection(BMX160* bmx160,
                                           AxisOrthoOrientation rotation)
    : bmx160(bmx160), rotation(rotation)
{
    accelerometerCorrector.fromFile("/sd/bmx160_accelerometer_correction.csv");
    magnetometerCorrector.fromFile("/sd/bmx160_magnetometer_correction.csv");

    // TODO: Remove
    // magnetometerCorrector.setA(Vector3f{0.73726, 0.59599, 2.27584});
    // magnetometerCorrector.setb(Vector3f{39.22325, -17.47903, -13.81505});

    gyroscopeBias = Vector3f{0, 0, 0};
}

bool BMX160WithCorrection::init() { return true; }

bool BMX160WithCorrection::selfTest() { return true; }

void BMX160WithCorrection::startCalibration()
{
    gyroscopeBias = Vector3f{0, 0, 0};
    calibrating   = true;
}

void BMX160WithCorrection::stopCalibration() { calibrating = false; }

BMX160WithCorrectionData BMX160WithCorrection::sampleImpl()
{
    if (!bmx160)
    {
        LOG_ERR(logger, "Driver doesn't point to valid sensor");
        return BMX160WithCorrectionData{};
    }

    Eigen::Vector3f avgAccel{0, 0, 0}, avgMag{0, 0, 0}, avgGyro{0, 0, 0}, vec;
    BMX160Data fifoElement;
    BMX160WithCorrectionData result;

    uint8_t fifoSize = bmx160->getLastFifoSize();

    // Read all data in the fifo
    for (int i = 0; i < fifoSize; i++)
    {
        fifoElement = bmx160->getFifoElement(i);

        // Read acceleration data
        static_cast<AccelerometerData>(fifoElement) >> vec;
        avgAccel += vec;

        // Read magnetometer data
        static_cast<MagnetometerData>(fifoElement) >> vec;
        avgMag += vec;

        // Read gyroscope data
        static_cast<GyroscopeData>(fifoElement) >> vec;
        avgGyro += vec;
    }

    // Average the samples
    if (fifoSize == 0)
    {
        static_cast<AccelerometerData>(bmx160->getLastSample()) >> avgAccel;
        static_cast<MagnetometerData>(bmx160->getLastSample()) >> avgMag;
        static_cast<GyroscopeData>(bmx160->getLastSample()) >> avgGyro;
        fifoElement = bmx160->getLastSample();
    }
    else
    {
        avgAccel /= fifoSize;
        avgMag /= fifoSize;
        avgGyro /= fifoSize;
    }

    // Correct the measurements
    avgAccel = accelerometerCorrector.correct(avgAccel);
    avgMag   = magnetometerCorrector.correct(avgMag);

    result.accelerationTimestamp = fifoElement.accelerationTimestamp;
    static_cast<AccelerometerData&>(result) << avgAccel;
    result.magneticFieldTimestamp = fifoElement.accelerationTimestamp;
    static_cast<MagnetometerData&>(result) << avgMag;
    result.angularVelocityTimestamp = fifoElement.accelerationTimestamp;
    static_cast<GyroscopeData&>(result) << avgGyro;

    if (calibrating)
    {
        gyroscopeBias = (gyroscopeBias * calibrationPoints + avgGyro) /
                        (calibrationPoints + 1);
        calibrationPoints++;
    }

    return rotateAxis(result);
}

BMX160WithCorrectionData BMX160WithCorrection::rotateAxis(
    BMX160WithCorrectionData data)
{
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
