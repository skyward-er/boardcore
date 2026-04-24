/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Tommaso Lamon
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


#include <algorithms/ANAS/ANAS0_types.h>

#include <reflect.hpp>


struct ANASOutData
{
    uint64_t timestamp;
    ANAS0_types_h_::NASOut ANASOut;

    ANASOutData() : timestamp(0), ANASOut() {};

    ANASOutData(uint64_t timestamp, ANAS0_types_h_::NASOut anasOut)
        : timestamp(timestamp), ANASOut(anasOut) {};

    // Ci sono due timestamp, uno interno (dell'alg) e uno esterno
    // (timestamptimer)

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ANASOutData,
                          FIELD_DEF(timestamp) FIELD_DEF2(ANASOut, Position)
                              FIELD_DEF2(ANASOut, Velocity)
                                  FIELD_DEF2(ANASOut, Quaternion)
                                      FIELD_DEF2(ANASOut, Timestamp));
    }
};


struct ANASLogsData
{
    uint64_t timestamp;
    ANAS0_types_h_::NASLogs ANASLogs;

    ANASLogsData() : timestamp(0), ANASLogs() {};

    ANASLogsData(uint64_t timestamp, ANAS0_types_h_::NASLogs anasLogs)
        : timestamp(timestamp), ANASLogs(anasLogs) {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            ANASLogsData,
            FIELD_DEF(timestamp) FIELD_DEF2(ANASLogs, Velocity) FIELD_DEF2(
                ANASLogs, Quaternion) FIELD_DEF2(ANASLogs, Covariance)
                FIELD_DEF2(ANASLogs, BaroActivation) FIELD_DEF2(
                    ANASLogs, GPSActivation) FIELD_DEF2(ANASLogs, MagActivation)
                    FIELD_DEF2(ANASLogs, AccActivation)
                        FIELD_DEF2(ANASLogs, Position));
    }
};


// Non so se logghiamo la LinearCovariance