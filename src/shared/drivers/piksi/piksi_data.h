/* Copyright (c) 2017-2019 Skyward Experimental Rocketry
 * Authors: Federico Terraneo
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once
#include <string>
#include <ostream>

/**
 * The GPS information
 */
struct GPSData
{
    /// timestamp in ms (anakin time, not GPS time). getTick()-timestamp tells
    /// you how "old" the data is.
    long long timestamp;

    bool fix;
    double latitude;      ///< [deg] //TODO: cast to float??
    double longitude;     ///< [deg] //TODO: cast to float??
    double height;        ///< [m]   //TODO: cast to float??
    float velocityNorth;  ///< [m/s]
    float velocityEast;   ///< [m/s]
    float velocityDown;   ///< [m/s]
    float speed;          ///< [m/s]
    int numSatellites;    ///< [1]
    static std::string header()
    {
        return "timestamp,fix,latitude,longitude,height,velocityNorth,velocityEast,velocityDown,speed,numSatellites\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << fix << "," << latitude << ","
           << longitude << "," << height <<"," << velocityNorth << "," << velocityEast << "," << velocityDown << ","
           << speed << "," << numSatellites << "\n";
    }
};