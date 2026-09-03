/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
#include <cstdint>
#include <type_traits>

template<typename T>
inline uint64_t discretizeRange(T value, T min, T max, unsigned int resolution)
{
    static_assert(std::is_arithmetic<T>::value, "T must be an arithmetic type (float, int, etc)");

    if (value < min)
    {
        return 0;
    }
    else if (value > max)
    {
        return resolution - 1;
    }

    return static_cast<uint64_t>((value - min) * resolution / (max - min));

}

template <typename T>
inline T undiscretizeRange(uint64_t value, T min, T max,
                           unsigned int resolution)
{
    static_assert(std::is_arithmetic<T>::value, "T must be an arithmetic type (float, int, etc)");

    return static_cast<T>(value * (max - min) / resolution + min);
}



class HighRateTMConversion
{
public:
    
    static long long bitsToTimestamp(uint64_t bits)
    {
        return (long long)bits;
    }

    static uint64_t timestampToBits(long long timestamp)
    {
        return (uint64_t)timestamp;
    }


    static float bitsToPressureAda(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, 50000.0, 105000.0, 4096);
    }

    static uint64_t pressureAdaToBits(float pressure_ada)
    {
        return discretizeRange<float>(pressure_ada, 50000.0, 105000.0, 4096);
    }


    static float bitsToPressureDigi(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, 50000.0, 105000.0, 8192);
    }

    static uint64_t pressureDigiToBits(float pressure_digi)
    {
        return discretizeRange<float>(pressure_digi, 50000.0, 105000.0, 8192);
    }


    static float bitsToMslAltitude(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, 0.0, 4096.0, 4096);
    }

    static uint64_t mslAltitudeToBits(float msl_altitude)
    {
        return discretizeRange<float>(msl_altitude, 0.0, 4096.0, 4096);
    }


    static float bitsToAglAltitude(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, -100.0, 3000.0, 512);
    }

    static uint64_t aglAltitudeToBits(float agl_altitude)
    {
        return discretizeRange<float>(agl_altitude, -100.0, 3000.0, 512);
    }


    static float bitsToVertSpeed(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, -300.0, 300.0, 1024);
    }

    static uint64_t vertSpeedToBits(float vert_speed)
    {
        return discretizeRange<float>(vert_speed, -300.0, 300.0, 1024);
    }


    static float bitsToVertSpeed2(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, -256.0, 256.0, 1024);
    }

    static uint64_t vertSpeed2ToBits(float vert_speed_2)
    {
        return discretizeRange<float>(vert_speed_2, -256.0, 256.0, 1024);
    }


    static float bitsToAccX(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, -157.0, 157.0, 2048);
    }

    static uint64_t accXToBits(float acc_x)
    {
        return discretizeRange<float>(acc_x, -157.0, 157.0, 2048);
    }


    static float bitsToAccY(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, -157.0, 157.0, 2048);
    }

    static uint64_t accYToBits(float acc_y)
    {
        return discretizeRange<float>(acc_y, -157.0, 157.0, 2048);
    }


    static float bitsToAccZ(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, -157.0, 157.0, 2048);
    }

    static uint64_t accZToBits(float acc_z)
    {
        return discretizeRange<float>(acc_z, -157.0, 157.0, 2048);
    }


    static float bitsToGyroX(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, -2000.0, 2000.0, 2048);
    }

    static uint64_t gyroXToBits(float gyro_x)
    {
        return discretizeRange<float>(gyro_x, -2000.0, 2000.0, 2048);
    }


    static float bitsToGyroY(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, -2000.0, 2000.0, 2048);
    }

    static uint64_t gyroYToBits(float gyro_y)
    {
        return discretizeRange<float>(gyro_y, -2000.0, 2000.0, 2048);
    }


    static float bitsToGyroZ(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, -2000.0, 2000.0, 2048);
    }

    static uint64_t gyroZToBits(float gyro_z)
    {
        return discretizeRange<float>(gyro_z, -2000.0, 2000.0, 2048);
    }


    static double bitsToGpsLat(uint64_t bits)
    {
        return undiscretizeRange<double>(bits, 41.777944, 41.835281, 2048);
    }

    static uint64_t gpsLatToBits(double gps_lat)
    {
        return discretizeRange<double>(gps_lat, 41.777944, 41.835281, 2048);
    }


    static double bitsToGpsLon(uint64_t bits)
    {
        return undiscretizeRange<double>(bits, 14.019528, 14.094634, 2048);
    }

    static uint64_t gpsLonToBits(double gps_lon)
    {
        return discretizeRange<double>(gps_lon, 14.019528, 14.094634, 2048);
    }


    static float bitsToGpsAlt(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, 0.0, 5120.0, 1024);
    }

    static uint64_t gpsAltToBits(float gps_alt)
    {
        return discretizeRange<float>(gps_alt, 0.0, 5120.0, 1024);
    }


    static float bitsToTemperature(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, -10.0, 70.0, 128);
    }

    static uint64_t temperatureToBits(float temperature)
    {
        return discretizeRange<float>(temperature, -10.0, 70.0, 128);
    }


    static uint8_t bitsToFmmState(uint64_t bits)
    {
        return (uint8_t)bits;
    }

    static uint64_t fmmStateToBits(uint8_t fmm_state)
    {
        return (uint64_t)fmm_state;
    }


    static uint8_t bitsToDplState(uint64_t bits)
    {
        return (uint8_t)bits;
    }

    static uint64_t dplStateToBits(uint8_t dpl_state)
    {
        return (uint64_t)dpl_state;
    }


    static uint8_t bitsToPinLaunch(uint64_t bits)
    {
        return (uint8_t)bits;
    }

    static uint64_t pinLaunchToBits(uint8_t pin_launch)
    {
        return (uint64_t)pin_launch;
    }


    static uint8_t bitsToPinNosecone(uint64_t bits)
    {
        return (uint8_t)bits;
    }

    static uint64_t pinNoseconeToBits(uint8_t pin_nosecone)
    {
        return (uint64_t)pin_nosecone;
    }


    static uint8_t bitsToGpsFix(uint64_t bits)
    {
        return (uint8_t)bits;
    }

    static uint64_t gpsFixToBits(uint8_t gps_fix)
    {
        return (uint64_t)gps_fix;
    }


};



class LowRateTMConversion
{
public:
    
    static long long bitsToLiftoffTs(uint64_t bits)
    {
        return (long long)bits;
    }

    static uint64_t liftoffTsToBits(long long liftoff_ts)
    {
        return (uint64_t)liftoff_ts;
    }


    static long long bitsToLiftoffMaxAccTs(uint64_t bits)
    {
        return (long long)bits;
    }

    static uint64_t liftoffMaxAccTsToBits(long long liftoff_max_acc_ts)
    {
        return (uint64_t)liftoff_max_acc_ts;
    }


    static float bitsToLiftoffMaxAcc(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, -157.0, 157.0, 2048);
    }

    static uint64_t liftoffMaxAccToBits(float liftoff_max_acc)
    {
        return discretizeRange<float>(liftoff_max_acc, -157.0, 157.0, 2048);
    }


    static long long bitsToMaxZspeedTs(uint64_t bits)
    {
        return (long long)bits;
    }

    static uint64_t maxZspeedTsToBits(long long max_zspeed_ts)
    {
        return (uint64_t)max_zspeed_ts;
    }


    static float bitsToMaxZspeed(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, -512.0, 512.0, 1024);
    }

    static uint64_t maxZspeedToBits(float max_zspeed)
    {
        return discretizeRange<float>(max_zspeed, -512.0, 512.0, 1024);
    }


    static float bitsToMaxSpeedAltitude(uint64_t bits)
    {
        return (float)bits;
    }

    static uint64_t maxSpeedAltitudeToBits(float max_speed_altitude)
    {
        return (uint64_t)max_speed_altitude;
    }


    static long long bitsToApogeeTs(uint64_t bits)
    {
        return (long long)bits;
    }

    static uint64_t apogeeTsToBits(long long apogee_ts)
    {
        return (uint64_t)apogee_ts;
    }


    static float bitsToNxpMinPressure(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, 50000.0, 100000.0, 4096);
    }

    static uint64_t nxpMinPressureToBits(float nxp_min_pressure)
    {
        return discretizeRange<float>(nxp_min_pressure, 50000.0, 100000.0, 4096);
    }


    static float bitsToHwMinPressure(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, 50000.0, 100000.0, 4096);
    }

    static uint64_t hwMinPressureToBits(float hw_min_pressure)
    {
        return discretizeRange<float>(hw_min_pressure, 50000.0, 100000.0, 4096);
    }


    static float bitsToKalmanMinPressure(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, 50000.0, 100000.0, 4096);
    }

    static uint64_t kalmanMinPressureToBits(float kalman_min_pressure)
    {
        return discretizeRange<float>(kalman_min_pressure, 50000.0, 100000.0, 4096);
    }


    static float bitsToDigitalMinPressure(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, 50000.0, 100000.0, 8192);
    }

    static uint64_t digitalMinPressureToBits(float digital_min_pressure)
    {
        return discretizeRange<float>(digital_min_pressure, 50000.0, 100000.0, 8192);
    }


    static float bitsToBaroMaxAltitutde(uint64_t bits)
    {
        return (float)bits;
    }

    static uint64_t baroMaxAltitutdeToBits(float baro_max_altitutde )
    {
        return (uint64_t)baro_max_altitutde ;
    }


    static float bitsToGpsMaxAltitude(uint64_t bits)
    {
        return (float)bits;
    }

    static uint64_t gpsMaxAltitudeToBits(float gps_max_altitude)
    {
        return (uint64_t)gps_max_altitude;
    }


    static float bitsToApogeeLat(uint64_t bits)
    {
        return (float)bits;
    }

    static uint64_t apogeeLatToBits(float apogee_lat)
    {
        return (uint64_t)apogee_lat;
    }


    static float bitsToApogeeLon(uint64_t bits)
    {
        return (float)bits;
    }

    static uint64_t apogeeLonToBits(float apogee_lon)
    {
        return (uint64_t)apogee_lon;
    }


    static long long bitsToDrogueDplTs(uint64_t bits)
    {
        return (long long)bits;
    }

    static uint64_t drogueDplTsToBits(long long drogue_dpl_ts)
    {
        return (uint64_t)drogue_dpl_ts;
    }


    static float bitsToDrogueDplMaxAcc(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, -157.0, 157.0, 2048);
    }

    static uint64_t drogueDplMaxAccToBits(float drogue_dpl_max_acc)
    {
        return discretizeRange<float>(drogue_dpl_max_acc, -157.0, 157.0, 2048);
    }


    static long long bitsToMainDplTs(uint64_t bits)
    {
        return (long long)bits;
    }

    static uint64_t mainDplTsToBits(long long main_dpl_ts)
    {
        return (uint64_t)main_dpl_ts;
    }


    static float bitsToMainDplAltitude(uint64_t bits)
    {
        return (float)bits;
    }

    static uint64_t mainDplAltitudeToBits(float main_dpl_altitude)
    {
        return (uint64_t)main_dpl_altitude;
    }


    static float bitsToMainDplZspeed(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, -512.0, 512.0, 1024);
    }

    static uint64_t mainDplZspeedToBits(float main_dpl_zspeed)
    {
        return discretizeRange<float>(main_dpl_zspeed, -512.0, 512.0, 1024);
    }


    static float bitsToMainDplAcc(uint64_t bits)
    {
        return undiscretizeRange<float>(bits, -16.0, 16.0, 2048);
    }

    static uint64_t mainDplAccToBits(float main_dpl_acc)
    {
        return discretizeRange<float>(main_dpl_acc, -16.0, 16.0, 2048);
    }


};


