/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Conterio, Alberto Nidasio
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

#include <sensors/BMX160/BMX160Data.h>

namespace Boardcore
{

struct BMX160WithCorrectionData : public BMX160Data
{
    BMX160WithCorrectionData() : BMX160Data() {}

    explicit BMX160WithCorrectionData(const BMX160Data& data)
        : BMX160WithCorrectionData(data, data, data)
    {
    }

    BMX160WithCorrectionData(AccelerometerData acc, GyroscopeData gyr,
                             MagnetometerData mag)
        : BMX160Data(acc, gyr, mag)
    {
    }

    BMX160WithCorrectionData& operator=(AccelerometerData acc)
    {
        accelerationX = acc.accelerationX;
        accelerationY = acc.accelerationY;
        accelerationZ = acc.accelerationZ;
        return *this;
    }

    BMX160WithCorrectionData& operator=(GyroscopeData gyr)
    {
        angularSpeedX = gyr.angularSpeedX;
        angularSpeedY = gyr.angularSpeedY;
        angularSpeedZ = gyr.angularSpeedZ;
        return *this;
    }

    BMX160WithCorrectionData& operator=(MagnetometerData mag)
    {
        magneticFieldX = mag.magneticFieldX;
        magneticFieldY = mag.magneticFieldY;
        magneticFieldZ = mag.magneticFieldZ;
        return *this;
    }

    static std::string header()
    {
        return "accelerationTimestamp,accelerationX,accelerationY,"
               "accelerationZ,angularSpeedTimestamp,angularSpeedX,"
               "angularSpeedY,angularSpeedZ,magneticFieldTimestamp,"
               "magneticFieldX,magneticFieldY,magneticFieldZ\n";
    }

    void print(std::ostream& os) const
    {
        os << accelerationTimestamp << "," << accelerationX << ","
           << accelerationY << "," << accelerationZ << ","
           << angularSpeedTimestamp << "," << angularSpeedX << ","
           << angularSpeedY << "," << angularSpeedZ << ","
           << magneticFieldTimestamp << "," << magneticFieldX << ","
           << magneticFieldY << "," << magneticFieldZ << "\n";
    }
};

struct BMX160GyroscopeCalibrationBiases
{
    float bx;
    float by;
    float bz;

    BMX160GyroscopeCalibrationBiases() {}

    BMX160GyroscopeCalibrationBiases(float x, float y, float z)
        : bx(x), by(y), bz(z)
    {
    }

    static std::string header() { return "bias_x,bias_y,bias_z\n"; }

    void print(std::ostream& os) const
    {
        os << bx << "," << by << "," << bz << "\n";
    }
};

}  // namespace Boardcore
