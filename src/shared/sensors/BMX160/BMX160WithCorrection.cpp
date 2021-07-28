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

BMX160WithCorrection::BMX160WithCorrection(
    BMX160* bmx160_, BMX160CorrectionParameters correctionParameters)
    : bmx160(bmx160_), minGyroSamplesForCalibration(
                           correctionParameters.minGyroSamplesForCalibration)
{
    accelerometerCorrector << correctionParameters.accelParams;
    magnetometerCorrector << correctionParameters.magnetoParams;
}

BMX160WithCorrection::BMX160WithCorrection(BMX160* bmx160_) : bmx160(bmx160_) {}

bool BMX160WithCorrection::init() { return true; }

bool BMX160WithCorrection::selfTest() { return true; }

bool BMX160WithCorrection::calibrate()
{
    if (!bmx160)
    {
        TRACE(
            "[BMX160WithCorrection] Error: driver doesn't point to valid "
            "sensor.");
        return false;
    }

    int samplesCounter = 0;
    BiasCalibration<GyroscopeData> gyroscopeCalibrator;
    BMX160Data fifoElement;
    uint8_t fifoSize;
    uint64_t gyroTimestamp = 0;

    // Set reference vector
    gyroscopeCalibrator.setReferenceVector({0, 0, 0});

    // Read the fifo and feed the gyroscope data to the calibrator
    fifoSize = bmx160->getLastFifoSize();
    for (uint8_t i = 0; i < fifoSize; i++)
    {
        fifoElement = bmx160->getFifoElement(i);

        if (fifoElement.gyro_timestamp > gyroTimestamp)
        {
            gyroTimestamp = fifoElement.gyro_timestamp;
            gyroscopeCalibrator.feed(fifoElement);

            samplesCounter++;
        }
    }

    // Continues until the averaged samples are at least the amount specified in
    // the configuration
    while (samplesCounter < minGyroSamplesForCalibration)
    {
        // Wait for another sample
        miosix::Thread::sleep(100);

        // Read the fifo and feed the gyroscope data to the calibrator
        fifoSize = bmx160->getLastFifoSize();
        for (uint8_t i = 0; i < fifoSize; i++)
        {
            fifoElement = bmx160->getFifoElement(i);

            if (fifoElement.gyro_timestamp > gyroTimestamp)
            {
                gyroTimestamp = fifoElement.gyro_timestamp;
                gyroscopeCalibrator.feed(fifoElement);

                samplesCounter++;
            }
        }
    }

    // Compute and save the calibration results
    gyroscopeCorrector = gyroscopeCalibrator.computeResult();

    return true;
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
    // TODO: pausekernellock

    if (!bmx160)
    {
        TRACE("Error: driver doesn't point to valid sensor.");
        return BMX160WithCorrectionData{};
    }

    Vector3f avgAccel{0, 0, 0}, avgMag{0, 0, 0}, avgGyro{0, 0, 0}, vec;
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
        if (fifoElement.accel_timestamp > accelTimestamp)
        {
            static_cast<AccelerometerData>(fifoElement) >> vec;
            avgAccel += vec;

            accelTimestamp = fifoElement.accel_timestamp;
            numAccel++;
        }

        // Read magnetometer data
        if (fifoElement.mag_timestamp > magTimestamp)
        {
            static_cast<MagnetometerData>(fifoElement) >> vec;
            avgMag += vec;

            magTimestamp = fifoElement.mag_timestamp;
            numMag++;
        }

        // Read gyroscope data
        if (fifoElement.gyro_timestamp > gyroTimestamp)
        {
            static_cast<GyroscopeData>(fifoElement) >> vec;
            avgGyro += vec;

            gyroTimestamp = fifoElement.gyro_timestamp;
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
    AccelerometerData acc = accelerometerCorrector.correct(result);
    result                = acc;
    MagnetometerData mag  = magnetometerCorrector.correct(result);
    result                = mag;
    GyroscopeData gyro;
    {
        miosix::PauseKernelLock lock;
        gyro = gyroscopeCorrector.correct(result);
    }
    result = gyro;

    // Get the timestamp of the newest value in fifo
    result.accel_timestamp = fifoElement.accel_timestamp;
    result.mag_timestamp   = fifoElement.accel_timestamp;
    result.gyro_timestamp  = fifoElement.accel_timestamp;

    result = rotateAxis(result);

    return result;
}

BMX160WithCorrectionData BMX160WithCorrection::rotateAxis(
    BMX160WithCorrectionData data)
{
    // TODO: Use rotaton matrix

    // Copy timestamps
    BMX160WithCorrectionData temp;
    temp.accel_timestamp = data.accel_timestamp;
    temp.gyro_timestamp  = data.gyro_timestamp;
    temp.mag_timestamp   = data.mag_timestamp;

    // Sensor's Z is X in body frame
    temp.accel_z = data.accel_x;
    temp.gyro_z  = data.gyro_x;
    temp.mag_z   = data.mag_x;

    // Sensor's X is -Z in body frame
    temp.accel_x = -data.accel_z;
    temp.gyro_x  = -data.gyro_z;
    temp.mag_x   = -data.mag_z;

    // Sensor's Y is -Y in body frame
    temp.accel_y = -data.accel_y;
    temp.gyro_y  = -data.gyro_y;
    temp.mag_y   = -data.mag_y;

    return temp;
}
