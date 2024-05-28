/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Lorenzo Cucchi, Fabrizio Monti
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

/**
 * @brief The groups of binary data available from the sensor.
 */
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

/**
 * @brief Values used to select data for the binary output from group 1 (common
 * group).
 */
enum CommonGroup
{
    COMMONGROUP_NONE         = 0x0000,  ///< None.
    COMMONGROUP_TIMESTARTUP  = 0x0001,  ///< Time since startup (nanoseconds).
    COMMONGROUP_TIMEGPS      = 0x0002,  ///< Gps time.
    COMMONGROUP_TIMESYNCIN   = 0x0004,  ///< Time since last sync in trigger.
    COMMONGROUP_YAWPITCHROLL = 0x0008,  ///< Yaw pitch roll.
    COMMONGROUP_QUATERNION   = 0x0010,  ///< Quaternion.
    COMMONGROUP_ANGULARRATE  = 0x0020,  ///< Angular Rate.
    COMMONGROUP_POSITION     = 0x0040,  ///< Position.
    COMMONGROUP_VELOCITY     = 0x0080,  ///< Velocity.
    COMMONGROUP_ACCEL        = 0x0100,  ///< Acceleration compensated (body).
    COMMONGROUP_IMU = 0x0200,  ///< Uncompensated gyro and accelerometer.
    COMMONGROUP_MAGPRES =
        0x0400,  ///< Compensated magnetic, temperature and pressure.
    COMMONGROUP_DELTATHETA = 0x0800,  ///< Delta time, theta and velocity.
    COMMONGROUP_INSSTATUS  = 0x1000,  ///< Ins status.
    COMMONGROUP_SYNCINCNT  = 0x2000,  ///< SyncIn count.
    COMMONGROUP_TIMEGPSPPS = 0x4000   ///< Time since last GpsPps trigger.
};

/**
 * @brief Values used to select data for the binary output from group 2 (time
 * group).
 */
enum TimeGroup
{
    TIMEGROUP_NONE        = 0x0000,  ///< None.
    TIMEGROUP_TIMESTARTUP = 0x0001,  ///< Time since startup (nanoseconds).
    TIMEGROUP_TIMEGPS     = 0x0002,  ///< Gps time.
    TIMEGROUP_GPSTOW      = 0x0004,  ///< Time since start of gps week.
    TIMEGROUP_GPSWEEK     = 0x0008,  ///< Gps week.
    TIMEGROUP_TIMESYNCIN  = 0x0010,  ///< Time since last sync in trigger.
    TIMEGROUP_TIMEGPSPPS  = 0x0020,  ///< Time since last GpsPps trigger.
    TIMEGROUP_TIMEUTC     = 0x0040,  ///< UTC time.
    TIMEGROUP_SYNCINCNT   = 0x0080,  ///< Sync in trigger count.
    TIMEGROUP_SYNCOUTCNT  = 0x0100,  ///< Sync out trigger count.
    TIMEGROUP_TIMESTATUS  = 0x0200   ///< Time valid status flag.
};

/**
 * @brief Values used to select data for the binary output from group 3 (imu
 * group).
 */
enum ImuGroup
{
    IMUGROUP_NONE        = 0x0000,  ///< None.
    IMUGROUP_IMUSTATUS   = 0x0001,  ///< Imu status.
    IMUGROUP_UNCOMPMAG   = 0x0002,  ///< Uncompensated magnetic.
    IMUGROUP_UNCOMPACCEL = 0x0004,  ///< Uncompensated accelerometer.
    IMUGROUP_UNCOMPGYRO  = 0x0008,  ///< Uncompensated gyroscope.
    IMUGROUP_TEMP        = 0x0010,  ///< Temperature.
    IMUGROUP_PRES        = 0x0020,  ///< Pressure.
    IMUGROUP_DELTATHETA  = 0x0040,  ///< Delta theta angles.
    IMUGROUP_DELTAVEL    = 0x0080,  ///< Delta velocity.
    IMUGROUP_MAG         = 0x0100,  ///< Compensated magnetometer.
    IMUGROUP_ACCEL       = 0x0200,  ///< Compensated accelerometer.
    IMUGROUP_ANGULARRATE = 0x0400,  ///< Compensated gyroscope.
};

/**
 * @brief Values used to select data for the binary output from group 4
 * and 7 (gps and gps2 group).
 */
enum GpsGroup
{
    GPSGROUP_NONE    = 0x0000,  ///< None.
    GPSGROUP_UTC     = 0x0001,  ///< Gps UTC time.
    GPSGROUP_TOW     = 0x0002,  ///< Gps time of week.
    GPSGROUP_WEEK    = 0x0004,  ///< Gps week.
    GPSGROUP_NUMSATS = 0x0008,  ///< Number of tracked satellites.
    GPSGROUP_FIX     = 0x0010,  ///< Gps fix.
    GPSGROUP_POSLLA =
        0x0020,  ///< Gps position (latitude, longitude, altitude).
    GPSGROUP_POSECEF  = 0x0040,  ///< Gps position (ECEF).
    GPSGROUP_VELNED   = 0x0080,  ///< Gps velocity (NED).
    GPSGROUP_VELECEF  = 0x0100,  ///< Gps velocity (ECEF).
    GPSGROUP_POSU     = 0x0200,  ///< Position uncertainty (NED).
    GPSGROUP_VELU     = 0x0400,  ///< Velocity uncertainty.
    GPSGROUP_TIMEU    = 0x0800,  ///< Time uncertainty.
    GPSGROUP_TIMEINFO = 0x1000,  ///< Gps time status and leap seconds.
    GPSGROUP_DOP      = 0x2000,  ///< Dilution of precision values.
};

/**
 * @brief Values used to select data for the binary output from group 5
 * (attitude group).
 */
enum AttitudeGroup
{
    ATTITUDEGROUP_NONE         = 0x0000,  ///< None.
    ATTITUDEGROUP_VPESTATUS    = 0x0001,  ///< VpeStatus.
    ATTITUDEGROUP_YAWPITCHROLL = 0x0002,  ///< Yaw pitch roll.
    ATTITUDEGROUP_QUATERNION   = 0x0004,  ///< Quaternion.
    ATTITUDEGROUP_DCM          = 0x0008,  ///< Directional cosine matrix.
    ATTITUDEGROUP_MAGNED       = 0x0010,  ///< Compensated magnetic (NED).
    ATTITUDEGROUP_ACCELNED     = 0x0020,  ///< Compensated acceleration (NED).
    ATTITUDEGROUP_LINEARACCELBODY =
        0x0040,  ///< Compensated linear acceleration (no gravity).
    ATTITUDEGROUP_LINEARACCELNED =
        0x0080,  ///< Compensated linear acceleration (no gravity) (NED).
    ATTITUDEGROUP_YPRU = 0x0100,  ///< Yaw Pitch Roll uncertainty.
};

/**
 * @brief Values used to select data for the binary output from group 6 (ins
 * group).
 */
enum InsGroup
{
    INSGROUP_NONE      = 0x0000,  ///< None.
    INSGROUP_INSSTATUS = 0x0001,  ///< Status.
    INSGROUP_POSLLA    = 0x0002,  ///< Position (latitude, longitude, altitude).
    INSGROUP_POSECEF   = 0x0004,  ///< Position (ECEF).
    INSGROUP_VELBODY   = 0x0008,  ///< Velocity (body).
    INSGROUP_VELNED    = 0x0010,  ///< Velocity (NED).
    INSGROUP_VELECEF   = 0x0020,  ///< Velocity (ECEF).
    INSGROUP_MAGECEF   = 0x0040,  ///< Compensated magnetic (ECEF).
    INSGROUP_ACCELECEF = 0x0080,  ///< Compensated acceleration (ECEF).
    INSGROUP_LINEARACCELECEF =
        0x0100,              ///< Compensated linear acceleration (ECEF).
    INSGROUP_POSU = 0x0200,  ///< Position uncertainty.
    INSGROUP_VELU = 0x0400,  ///< Velocity uncertainty.
};

/**
 * @brief Structure to handle antenna A position units [m]
 */
struct AntennaPosition
{
    float posX;  // Relative position of GPS antenna (X-axis)
    float posY;
    float posZ;
    float uncX;  // Uncertainty in the X-axis position measurement
    float uncY;
    float uncZ;
};

/**
 * @brief Structure to handle INS data.
 */
struct Ins_Lla
{
    uint64_t insTimestamp;
    uint16_t fix_gps;
    uint16_t fix_ins;
    uint16_t status;
    float yaw;
    float pitch;
    float roll;
    float latitude;
    float longitude;
    float altitude;
    float nedVelX;
    float nedVelY;
    float nedVelZ;
};

/**
 * @brief Sample options (data output packets) available.
 */
enum class SampleOptions : uint8_t
{
    FULL,  ///< All data is sampled
    ARP,   ///< Only ARP needed data is sampled
};

/**
 * @brief Structure to handle binary message in case of full sampling (all
 * available measurements are taken).
 */
struct __attribute__((packed)) BinaryDataFull
{
    uint8_t group;
    uint16_t common;
    uint16_t gnss;
    float yaw_bin;
    float pitch_bin;
    float roll_bin;
    float quatX_bin;
    float quatY_bin;
    float quatZ_bin;
    float quatW_bin;
    float angx;
    float angy;
    float angz;
    float velx;
    float vely;
    float velz;
    float accx;
    float accy;
    float accz;
    float magx;
    float magy;
    float magz;
    float temp;
    float pres;
    uint16_t ins_status;
    uint8_t numsats;
    uint8_t fix;
    double latitude_bin;
    double longitude_bin;
    double altitude_bin;
    uint16_t checksum;
};

/**
 * @brief Structure to handle the custom data packet needed for the
 * automated antennas.
 * Selected data:
 * - Common group:
 *      - Yaw pitch and roll
 *      - Quaternion
 *      - Angular rate
 * - Gps1:
 *      - Fix
 *      - PosLla
 */
struct __attribute__((packed)) BinaryDataArp
{
    uint8_t groupByte;
    uint16_t groupField1;
    uint16_t groupField2;
    float yaw;
    float pitch;
    float roll;
    float quaternionX;
    float quaternionY;
    float quaternionZ;
    float quaternionW;
    float angularX;
    float angularY;
    float angularZ;
    uint8_t gpsFix;
    double latitude;
    double longitude;
    double altitude;
    uint16_t checksum;
};

}  // namespace VN300Defs

}  // namespace Boardcore
