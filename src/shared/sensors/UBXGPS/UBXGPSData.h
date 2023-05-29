/* Copyright (c) 2020-2022 Skyward Experimental Rocketry
 * Authors: Davide Bonomini, Damiano Amatruda
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

namespace Boardcore
{

/**
 * @brief Structure to handle UBX UTC time.
 * Nanoseconds range from -5000000 (5 ms) to 994999999 (~995 ms) and when
 * negative the other fields have been rounded to the nearest hundredth of a
 * second.
 * Because of leap seconds, minutes can be a second longer or shorter, so
 * seconds range from 0 to 60.
 */
struct UBXDateTime
{
    uint16_t year      = 0;  // Year (UTC) [y]
    uint8_t month      = 0;  // Month, range 1..12 (UTC) [month]
    uint8_t day        = 0;  // Day of month, range 1..31 (UTC) [d]
    uint8_t hour       = 0;  // Hour of day, range 0..23 (UTC) [h]
    uint8_t minute     = 0;  // Minute of hour, range 0..59 (UTC) [min]
    uint8_t second     = 0;  // Seconds of minute, range 0..60 (UTC) [s]
    int32_t nanosecond = 0;  // Fraction of second, range -1e9 .. 1e9 (UTC) [ns]
    uint32_t accuracy  = 0;  // Time accuracy estimate (UTC) [ns]

    static std::string header()
    {
        return "year,month,day,hour,minute,second,nanosecond,accuracy";
    }

    void print(std::ostream &os) const
    {
        os << year << "," << (int)month << "," << (int)day << "," << (int)hour
           << "," << (int)minute << "," << (int)second << "," << nanosecond
           << "," << accuracy;
    }
};

struct UBXGPSData : public GPSData
{
    UBXDateTime ubxTime;

    static std::string header()
    {
        return "gpsTimestamp,latitude,longitude,height,velocityNorth,"
               "velocityEast,velocityDown,speed,track,positionDOP,satellites,"
               "fix," +
               UBXDateTime::header() + "\n";
    }

    void print(std::ostream &os) const
    {
        os << gpsTimestamp << "," << latitude << "," << longitude << ","
           << height << "," << velocityNorth << "," << velocityEast << ","
           << velocityDown << "," << speed << "," << track << "," << positionDOP
           << "," << (int)satellites << "," << (int)fix << ",";

        ubxTime.print(os);

        os << "\n";
    }
};

}  // namespace Boardcore
