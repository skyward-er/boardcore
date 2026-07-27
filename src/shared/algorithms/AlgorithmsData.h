/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Pietro Bortolus
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

#include <abk/ABK_types.h>
#include <anas/ANAS0_types.h>
#include <nasdaq/NASDAQ0_types.h>
#include <prf/PRF_types.h>
#include <sda/SDA_types.h>

#include <cstdint>
#include <ostream>
#include <reflect.hpp>
#include <string>

namespace Boardcore
{

struct ANASState
{
    uint64_t timestamp = 0;

    // Position [m]
    float n = 0;  ///< North (x)
    float e = 0;  ///< East  (y)
    float d = 0;  ///< Down  (z)

    // Velocity [m/s]
    float vn = 0;  ///< Velocity North (x)
    float ve = 0;  ///< Velocity East  (y)
    float vd = 0;  ///< Velocity Down  (z)

    // Attitude as quaternion, from body to NED frame
    float qx = 0;  ///< Quaternion x
    float qy = 0;  ///< Quaternion y
    float qz = 0;  ///< Quaternion z
    float qw = 1;  ///< Quaternion w

    ANASState() : timestamp(0) {};

    ANASState(uint64_t timestamp, const float position[3],
              const float velocity[3], const float quaternions[4])
        : timestamp(timestamp), n(position[0]), e(position[1]), d(position[2]),
          vn(velocity[0]), ve(velocity[1]), vd(velocity[2]), qx(quaternions[0]),
          qy(quaternions[1]), qz(quaternions[2]), qw(quaternions[3]) {};

    ANASState(uint64_t timestamp, const ANAS0_types_h_::ANASOut& anasOut)
        : timestamp(timestamp), n(anasOut.Position[0]), e(anasOut.Position[1]),
          d(anasOut.Position[2]), vn(anasOut.Velocity[0]),
          ve(anasOut.Velocity[1]), vd(anasOut.Velocity[2]),
          qx(anasOut.Quaternion[0]), qy(anasOut.Quaternion[1]),
          qz(anasOut.Quaternion[2]), qw(anasOut.Quaternion[3]) {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ANASState,
                          FIELD_DEF(timestamp) FIELD_DEF(n) FIELD_DEF(e)
                              FIELD_DEF(d) FIELD_DEF(vn) FIELD_DEF(ve)
                                  FIELD_DEF(vd) FIELD_DEF(qx) FIELD_DEF(qy)
                                      FIELD_DEF(qz) FIELD_DEF(qw));
    }
};

struct ANASLogsData
{
    ANAS0_types_h_::ANASLogs ANASLogs;

    ANASLogsData() : ANASLogs() {};

    ANASLogsData(ANAS0_types_h_::ANASLogs anasLogs) : ANASLogs(anasLogs) {};

    ANASLogsData(uint64_t timestamp, ANAS0_types_h_::ANASLogs anasLogs)
        : ANASLogs(anasLogs)
    {
        ANASLogs.Timestamp = timestamp;
    };

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            ANASLogsData,
            FIELD_DEF2(ANASLogs, Timestamp) FIELD_DEF2(ANASLogs, Position)
                FIELD_DEF2(ANASLogs, Velocity) FIELD_DEF2(ANASLogs, Quaternion)
                    FIELD_DEF2(ANASLogs, CovMatD) FIELD_DEF2(
                        ANASLogs, BaroPitotAct) FIELD_DEF2(ANASLogs, GPSAct)
                        FIELD_DEF2(ANASLogs, MagAct)
                            FIELD_DEF2(ANASLogs, AccAct));
    }
};

struct NASDAQState
{
    uint64_t timestamp = 0;

    // Position
    float n = 0;
    float e = 0;
    float d = 0;

    // Velocity
    float vn = 0;
    float ve = 0;
    float vd = 0;

    NASDAQState() : timestamp(0) {};

    NASDAQState(uint64_t timestamp, const float position[3],
                const float velocity[3])
        : timestamp(timestamp), n(position[0]), e(position[1]), d(position[2]),
          vn(velocity[0]), ve(velocity[1]), vd(velocity[2]) {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(NASDAQState,
                          FIELD_DEF(timestamp) FIELD_DEF(n) FIELD_DEF(e)
                              FIELD_DEF(d) FIELD_DEF(vn) FIELD_DEF(ve)
                                  FIELD_DEF(vd));
    }
};

struct NASDAQLogsWrapper
{
    uint64_t obswTimestamp;

    NASDAQ0_types_h_::NASDAQLogs NASDAQLog;

    NASDAQLogsWrapper() : obswTimestamp(0) {};

    NASDAQLogsWrapper(uint64_t timestamp,
                      const NASDAQ0_types_h_::NASDAQLogs logs)
        : obswTimestamp(timestamp), NASDAQLog(logs) {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            NASDAQLogsWrapper,
            FIELD_DEF2(NASDAQLog, Timestamp) FIELD_DEF2(NASDAQLog, Position)
                FIELD_DEF2(NASDAQLog, Velocity) FIELD_DEF2(NASDAQLog, CovMatD)
                    FIELD_DEF2(NASDAQLog, BaroAct) FIELD_DEF2(NASDAQLog, GPSAct)
                        FIELD_DEF2(NASDAQLog, ADAAct));
    }
};

struct NASState
{
    uint64_t timestamp = 0;

    float n = 0;  ///< North (x)
    float e = 0;  ///< East  (y)
    float d = 0;  ///< Down  (z)

    float vn = 0;  ///< Velocity North (x)
    float ve = 0;  ///< Velocity East  (y)
    float vd = 0;  ///< Velocity Down  (z)

    NASState() : timestamp(0) {};

    NASState(NASDAQState nasdaqState)
    {
        timestamp = nasdaqState.timestamp;
        n         = nasdaqState.n;
        e         = nasdaqState.e;
        d         = nasdaqState.d;
        vn        = nasdaqState.vn;
        ve        = nasdaqState.ve;
        vd        = nasdaqState.vd;
    }

    NASState(ANASState anasState)
    {
        timestamp = anasState.timestamp;
        n         = anasState.n;
        e         = anasState.e;
        d         = anasState.d;
        vn        = anasState.vn;
        ve        = anasState.ve;
        vd        = anasState.vd;
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(NASState, FIELD_DEF(timestamp) FIELD_DEF(n)
                                        FIELD_DEF(e) FIELD_DEF(d) FIELD_DEF(vn)
                                            FIELD_DEF(ve) FIELD_DEF(vd));
    }
};

struct ABKLogsData
{
    uint64_t timestamp;
    ABK_types_h_::ABKLogs ABKLogs;

    ABKLogsData() : timestamp(0), ABKLogs() {};

    ABKLogsData(uint64_t timestamp, ABK_types_h_::ABKLogs abkLogs)
        : timestamp(timestamp), ABKLogs(abkLogs) {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            ABKLogsData, FIELD_DEF(timestamp) FIELD_DEF2(ABKLogs, ABKCommand)
                             FIELD_DEF2(ABKLogs, FilterCoefficient)
                                 FIELD_DEF2(ABKLogs, PrePIDCommand)
                                     FIELD_DEF2(ABKLogs, PostPIDCommand)
                                         FIELD_DEF2(ABKLogs, BypassActivation));
    }
};

struct SDALogsWrapper
{
    SDA_types_h_::SDALogs logs;

    SDALogsWrapper() : logs() {};

    SDALogsWrapper(SDA_types_h_::SDALogs logs) : logs(logs) {};

    SDALogsWrapper(SDA_types_h_::SDALogs logs, uint64_t timestamp) : logs(logs)
    {
        this->logs.Timestamp = timestamp;
    };

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            SDALogsWrapper,
            FIELD_DEF2(logs, ShutdownCommand) FIELD_DEF2(logs, Timestamp)
                FIELD_DEF2(logs, ShutdownCounter) FIELD_DEF2(logs, Apogee));
    }
};

struct WingControllerLogsData
{
    uint64_t timestamp;
    PRF_types_h_::PRFLogs PRFLogs;

    WingControllerLogsData() : timestamp(0), PRFLogs() {};

    WingControllerLogsData(uint64_t timestamp,
                           PRF_types_h_::PRFLogs wingControllerLogs)
        : timestamp(timestamp), PRFLogs(wingControllerLogs) {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            WingControllerLogsData,
            FIELD_DEF(timestamp) FIELD_DEF2(PRFLogs, Q1) FIELD_DEF2(PRFLogs, Q2)
                FIELD_DEF2(PRFLogs, TerminalTarget) FIELD_DEF2(
                    PRFLogs, TargetIndex) FIELD_DEF2(PRFLogs, Heading)
                    FIELD_DEF2(PRFLogs, Reference)
                        FIELD_DEF2(PRFLogs, ServoCommands)
                            FIELD_DEF2(PRFLogs, WindHeading));
    }
};

}  // namespace Boardcore
