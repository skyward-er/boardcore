/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Lorenzo Cucchi
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

namespace Boardcore
{

namespace VN300Defs
{

/// \brief The available binary output groups.
enum BinaryGroup
{
    BINARYGROUP_COMMON   = 0x01,  ///< Common group.
    BINARYGROUP_TIME     = 0x02,  ///< Time group.
    BINARYGROUP_IMU      = 0x04,  ///< IMU group.
    BINARYGROUP_GPS      = 0x08,  ///< GPS group.
    BINARYGROUP_ATTITUDE = 0x10,  ///< Attitude group.
    BINARYGROUP_INS      = 0x20,  ///< INS group.
    BINARYGROUP_GPS2     = 0x40   ///< GPS2 group.
};

/// \brief Flags for the binary group 1 'Common' in the binary output registers.
enum CommonGroup
{
    COMMONGROUP_NONE         = 0x0000,  ///< None.
    COMMONGROUP_TIMESTARTUP  = 0x0001,  ///< TimeStartup.
    COMMONGROUP_TIMEGPS      = 0x0002,  ///< TimeGps.
    COMMONGROUP_TIMESYNCIN   = 0x0004,  ///< TimeSyncIn.
    COMMONGROUP_YAWPITCHROLL = 0x0008,  ///< YawPitchRoll.
    COMMONGROUP_QUATERNION   = 0x0010,  ///< Quaternion.
    COMMONGROUP_ANGULARRATE  = 0x0020,  ///< AngularRate.
    COMMONGROUP_POSITION     = 0x0040,  ///< Position.
    COMMONGROUP_VELOCITY     = 0x0080,  ///< Velocity.
    COMMONGROUP_ACCEL        = 0x0100,  ///< Accel.
    COMMONGROUP_IMU          = 0x0200,  ///< Imu.
    COMMONGROUP_MAGPRES      = 0x0400,  ///< MagPres.
    COMMONGROUP_DELTATHETA   = 0x0800,  ///< DeltaTheta.
    COMMONGROUP_INSSTATUS    = 0x1000,  ///< InsStatus.
    COMMONGROUP_SYNCINCNT    = 0x2000,  ///< SyncInCnt.
    COMMONGROUP_TIMEGPSPPS   = 0x4000   ///< TimeGpsPps.
};

/// \brief Flags for the binary group 2 'Time' in the binary output registers.
enum TimeGroup
{
    TIMEGROUP_NONE        = 0x0000,  ///< None.
    TIMEGROUP_TIMESTARTUP = 0x0001,  ///< TimeStartup.
    TIMEGROUP_TIMEGPS     = 0x0002,  ///< TimeGps.
    TIMEGROUP_GPSTOW      = 0x0004,  ///< GpsTow.
    TIMEGROUP_GPSWEEK     = 0x0008,  ///< GpsWeek.
    TIMEGROUP_TIMESYNCIN  = 0x0010,  ///< TimeSyncIn.
    TIMEGROUP_TIMEGPSPPS  = 0x0020,  ///< TimeGpsPps.
    TIMEGROUP_TIMEUTC     = 0x0040,  ///< TimeUTC.
    TIMEGROUP_SYNCINCNT   = 0x0080,  ///< SyncInCnt.
    TIMEGROUP_SYNCOUTCNT  = 0x0100,  ///< SyncOutCnt.
    TIMEGROUP_TIMESTATUS  = 0x0200   ///< TimeStatus.
};

/// \brief Flags for the binary group 3 'IMU' in the binary output registers.
enum ImuGroup
{
    IMUGROUP_NONE        = 0x0000,  ///< None.
    IMUGROUP_IMUSTATUS   = 0x0001,  ///< ImuStatus.
    IMUGROUP_UNCOMPMAG   = 0x0002,  ///< UncompMag.
    IMUGROUP_UNCOMPACCEL = 0x0004,  ///< UncompAccel.
    IMUGROUP_UNCOMPGYRO  = 0x0008,  ///< UncompGyro.
    IMUGROUP_TEMP        = 0x0010,  ///< Temp.
    IMUGROUP_PRES        = 0x0020,  ///< Pres.
    IMUGROUP_DELTATHETA  = 0x0040,  ///< DeltaTheta.
    IMUGROUP_DELTAVEL    = 0x0080,  ///< DeltaVel.
    IMUGROUP_MAG         = 0x0100,  ///< Mag.
    IMUGROUP_ACCEL       = 0x0200,  ///< Accel.
    IMUGROUP_ANGULARRATE = 0x0400,  ///< AngularRate.
    IMUGROUP_SENSSAT     = 0x0800,  ///< SensSat.
};

/// \brief Flags for the binary group 4 'GPS' and group 7 'GPS2' in the binary
/// output registers.
enum GpsGroup
{
    GPSGROUP_NONE     = 0x0000,  ///< None.
    GPSGROUP_UTC      = 0x0001,  ///< UTC.
    GPSGROUP_TOW      = 0x0002,  ///< Tow.
    GPSGROUP_WEEK     = 0x0004,  ///< Week.
    GPSGROUP_NUMSATS  = 0x0008,  ///< NumSats.
    GPSGROUP_FIX      = 0x0010,  ///< Fix.
    GPSGROUP_POSLLA   = 0x0020,  ///< PosLla.
    GPSGROUP_POSECEF  = 0x0040,  ///< PosEcef.
    GPSGROUP_VELNED   = 0x0080,  ///< VelNed.
    GPSGROUP_VELECEF  = 0x0100,  ///< VelEcef.
    GPSGROUP_POSU     = 0x0200,  ///< PosU.
    GPSGROUP_VELU     = 0x0400,  ///< VelU.
    GPSGROUP_TIMEU    = 0x0800,  ///< TimeU.
    GPSGROUP_TIMEINFO = 0x1000,  ///< TimeInfo.
    GPSGROUP_DOP      = 0x2000,  ///< Dop.
};

/// \brief Flags for the binary group 5 'Attitude' in the binary output
/// registers.
enum AttitudeGroup
{
    ATTITUDEGROUP_NONE            = 0x0000,  ///< None.
    ATTITUDEGROUP_VPESTATUS       = 0x0001,  ///< VpeStatus.
    ATTITUDEGROUP_YAWPITCHROLL    = 0x0002,  ///< YawPitchRoll.
    ATTITUDEGROUP_QUATERNION      = 0x0004,  ///< Quaternion.
    ATTITUDEGROUP_DCM             = 0x0008,  ///< DCM.
    ATTITUDEGROUP_MAGNED          = 0x0010,  ///< MagNed.
    ATTITUDEGROUP_ACCELNED        = 0x0020,  ///< AccelNed.
    ATTITUDEGROUP_LINEARACCELBODY = 0x0040,  ///< LinearAccelBody.
    ATTITUDEGROUP_LINEARACCELNED  = 0x0080,  ///< LinearAccelNed.
    ATTITUDEGROUP_YPRU            = 0x0100,  ///< YprU.
    ATTITUDEGROUP_HEAVE           = 0x1000,  ///< Heave.
};

/// \brief Flags for the binary group 6 'INS' in the binary output registers.
enum InsGroup
{
    INSGROUP_NONE            = 0x0000,  ///< None.
    INSGROUP_INSSTATUS       = 0x0001,  ///< InsStatus.
    INSGROUP_POSLLA          = 0x0002,  ///< PosLla.
    INSGROUP_POSECEF         = 0x0004,  ///< PosEcef.
    INSGROUP_VELBODY         = 0x0008,  ///< VelBody.
    INSGROUP_VELNED          = 0x0010,  ///< VelNed.
    INSGROUP_VELECEF         = 0x0020,  ///< VelEcef.
    INSGROUP_MAGECEF         = 0x0040,  ///< MagEcef.
    INSGROUP_ACCELECEF       = 0x0080,  ///< AccelEcef.
    INSGROUP_LINEARACCELECEF = 0x0100,  ///< LinearAccelEcef.
    INSGROUP_POSU            = 0x0200,  ///< PosU.
    INSGROUP_VELU            = 0x0400,  ///< VelU.
};

const unsigned char
    BinaryGroupLengths[sizeof(uint8_t) * 8][sizeof(uint16_t) * 15] = {
        {8, 8, 8, 12, 16, 12, 24, 12, 12, 24, 20, 28, 2, 4, 8},     // Group 1
        {8, 8, 8, 2, 8, 8, 8, 4, 4, 1, 0, 0, 0, 0, 0},              // Group 2
        {2, 12, 12, 12, 4, 4, 16, 12, 12, 12, 12, 2, 40, 0, 0},     // Group 3
        {8, 8, 2, 1, 1, 24, 24, 12, 12, 12, 4, 4, 2, 28, 0},        // Group 4
        {2, 12, 16, 36, 12, 12, 12, 12, 12, 12, 28, 24, 12, 0, 0},  // Group 5
        {2, 24, 24, 12, 12, 12, 12, 12, 12, 4, 4, 68, 64, 0, 0},    // Group 6
        {8, 8, 2, 1, 1, 24, 24, 12, 12, 12, 4, 4, 2, 28, 0},        // Group 7
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}  // Invalid group
};

}  // namespace VN300Defs

}  // namespace Boardcore
