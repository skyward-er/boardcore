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

#include <logger/Logger.h>
#include <sensors/SensorData.h>

#include <userde.hpp>

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

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            UBXDateTime, FIELD_DEF(year) FIELD_DEF(month) FIELD_DEF(day)
                             FIELD_DEF(hour) FIELD_DEF(minute) FIELD_DEF(second)
                                 FIELD_DEF(nanosecond) FIELD_DEF(accuracy));
    }
};

struct UBXGPSData : public GPSData
{
    UBXDateTime ubxTime;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            Boardcore::UBXGPSData,
            EXTEND_DEF(GPSData) FIELD_DEF2(ubxTime, year)
                FIELD_DEF2(ubxTime, month) FIELD_DEF2(ubxTime, day) FIELD_DEF2(
                    ubxTime, hour) FIELD_DEF2(ubxTime, minute)
                    FIELD_DEF2(ubxTime, second) FIELD_DEF2(ubxTime, nanosecond)
                        FIELD_DEF2(ubxTime, accuracy));
    }
};

}  // namespace Boardcore

/* template <>
struct socrate::userde::Serde<Boardcore::UBXDateTime, void>
{
    static constexpr size_t size()
    {
        return sizeof(uint16_t) + sizeof(uint8_t) * 5 + sizeof(int32_t) +
               sizeof(uint32_t);
    }

    static void serialize(const Boardcore::UBXDateTime& value, Stream& stream)
    {
        stream.write(&value.year, sizeof(uint16_t));
        stream.write(&value.month, sizeof(uint8_t));
        stream.write(&value.day, sizeof(uint8_t));
        stream.write(&value.hour, sizeof(uint8_t));
        stream.write(&value.minute, sizeof(uint8_t));
        stream.write(&value.second, sizeof(uint8_t));
        stream.write(&value.nanosecond, sizeof(int32_t));
        stream.write(&value.accuracy, sizeof(uint32_t));
    }

    static void deserialize(Boardcore::UBXDateTime& value, Stream& stream)
    {
        stream.read(&value.year, sizeof(uint16_t));
        stream.read(&value.month, sizeof(uint8_t));
        stream.read(&value.day, sizeof(uint8_t));
        stream.read(&value.hour, sizeof(uint8_t));
        stream.read(&value.minute, sizeof(uint8_t));
        stream.read(&value.second, sizeof(uint8_t));
        stream.read(&value.nanosecond, sizeof(int32_t));
        stream.read(&value.accuracy, sizeof(uint32_t));
    }
};
 */
