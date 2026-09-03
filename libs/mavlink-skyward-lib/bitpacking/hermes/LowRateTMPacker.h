/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Conterio, Luca Erbetta
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

#include "bitpacking/BitPacker.h"
#include "mavlink_lib/hermes/mavlink.h"
#include "bitpacking/hermes/ConversionFunctions.h"

class LowRateTMPacker
{
public:
    static constexpr int LR_TM_PACKET_SIZE            = 40;
    static constexpr int LR_TM_SINGLE_PACKET_SIZE_BITS = 319;

    static_assert(MAVLINK_MSG_LR_TM_FIELD_PAYLOAD_LEN == LR_TM_PACKET_SIZE,
                  "Payload size mismatch! Mavlnk payload size differes from declared size. Maybe your mavlink definitions are outdated?");




    LowRateTMPacker(uint8_t *packet) : packet(packet), bitpacker(packet, LR_TM_PACKET_SIZE)
    {
    }

    
    bool packLiftoffTs(long long liftoff_ts, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::liftoffTsToBits(liftoff_ts);

        return bitpacker.pack(
            INDEX_LIFTOFF_TS + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_LIFTOFF_TS, bits);
    }

    bool packLiftoffMaxAccTs(long long liftoff_max_acc_ts, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::liftoffMaxAccTsToBits(liftoff_max_acc_ts);

        return bitpacker.pack(
            INDEX_LIFTOFF_MAX_ACC_TS + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_LIFTOFF_MAX_ACC_TS, bits);
    }

    bool packLiftoffMaxAcc(float liftoff_max_acc, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::liftoffMaxAccToBits(liftoff_max_acc);

        return bitpacker.pack(
            INDEX_LIFTOFF_MAX_ACC + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_LIFTOFF_MAX_ACC, bits);
    }

    bool packMaxZspeedTs(long long max_zspeed_ts, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::maxZspeedTsToBits(max_zspeed_ts);

        return bitpacker.pack(
            INDEX_MAX_ZSPEED_TS + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_MAX_ZSPEED_TS, bits);
    }

    bool packMaxZspeed(float max_zspeed, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::maxZspeedToBits(max_zspeed);

        return bitpacker.pack(
            INDEX_MAX_ZSPEED + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_MAX_ZSPEED, bits);
    }

    bool packMaxSpeedAltitude(float max_speed_altitude, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::maxSpeedAltitudeToBits(max_speed_altitude);

        return bitpacker.pack(
            INDEX_MAX_SPEED_ALTITUDE + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_MAX_SPEED_ALTITUDE, bits);
    }

    bool packApogeeTs(long long apogee_ts, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::apogeeTsToBits(apogee_ts);

        return bitpacker.pack(
            INDEX_APOGEE_TS + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_APOGEE_TS, bits);
    }

    bool packNxpMinPressure(float nxp_min_pressure, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::nxpMinPressureToBits(nxp_min_pressure);

        return bitpacker.pack(
            INDEX_NXP_MIN_PRESSURE + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_NXP_MIN_PRESSURE, bits);
    }

    bool packHwMinPressure(float hw_min_pressure, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::hwMinPressureToBits(hw_min_pressure);

        return bitpacker.pack(
            INDEX_HW_MIN_PRESSURE + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_HW_MIN_PRESSURE, bits);
    }

    bool packKalmanMinPressure(float kalman_min_pressure, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::kalmanMinPressureToBits(kalman_min_pressure);

        return bitpacker.pack(
            INDEX_KALMAN_MIN_PRESSURE + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_KALMAN_MIN_PRESSURE, bits);
    }

    bool packDigitalMinPressure(float digital_min_pressure, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::digitalMinPressureToBits(digital_min_pressure);

        return bitpacker.pack(
            INDEX_DIGITAL_MIN_PRESSURE + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_DIGITAL_MIN_PRESSURE, bits);
    }

    bool packBaroMaxAltitutde(float baro_max_altitutde , size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::baroMaxAltitutdeToBits(baro_max_altitutde );

        return bitpacker.pack(
            INDEX_BARO_MAX_ALTITUTDE  + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_BARO_MAX_ALTITUTDE , bits);
    }

    bool packGpsMaxAltitude(float gps_max_altitude, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::gpsMaxAltitudeToBits(gps_max_altitude);

        return bitpacker.pack(
            INDEX_GPS_MAX_ALTITUDE + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_GPS_MAX_ALTITUDE, bits);
    }

    bool packApogeeLat(float apogee_lat, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::apogeeLatToBits(apogee_lat);

        return bitpacker.pack(
            INDEX_APOGEE_LAT + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_APOGEE_LAT, bits);
    }

    bool packApogeeLon(float apogee_lon, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::apogeeLonToBits(apogee_lon);

        return bitpacker.pack(
            INDEX_APOGEE_LON + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_APOGEE_LON, bits);
    }

    bool packDrogueDplTs(long long drogue_dpl_ts, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::drogueDplTsToBits(drogue_dpl_ts);

        return bitpacker.pack(
            INDEX_DROGUE_DPL_TS + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_DROGUE_DPL_TS, bits);
    }

    bool packDrogueDplMaxAcc(float drogue_dpl_max_acc, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::drogueDplMaxAccToBits(drogue_dpl_max_acc);

        return bitpacker.pack(
            INDEX_DROGUE_DPL_MAX_ACC + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_DROGUE_DPL_MAX_ACC, bits);
    }

    bool packMainDplTs(long long main_dpl_ts, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::mainDplTsToBits(main_dpl_ts);

        return bitpacker.pack(
            INDEX_MAIN_DPL_TS + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_MAIN_DPL_TS, bits);
    }

    bool packMainDplAltitude(float main_dpl_altitude, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::mainDplAltitudeToBits(main_dpl_altitude);

        return bitpacker.pack(
            INDEX_MAIN_DPL_ALTITUDE + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_MAIN_DPL_ALTITUDE, bits);
    }

    bool packMainDplZspeed(float main_dpl_zspeed, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::mainDplZspeedToBits(main_dpl_zspeed);

        return bitpacker.pack(
            INDEX_MAIN_DPL_ZSPEED + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_MAIN_DPL_ZSPEED, bits);
    }

    bool packMainDplAcc(float main_dpl_acc, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = LowRateTMConversion::mainDplAccToBits(main_dpl_acc);

        return bitpacker.pack(
            INDEX_MAIN_DPL_ACC + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_MAIN_DPL_ACC, bits);
    }


    
    bool unpackLiftoffTs(long long* liftoff_ts, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_LIFTOFF_TS + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_LIFTOFF_TS, &bits);
        
        if(success)
        {
            *liftoff_ts = LowRateTMConversion::bitsToLiftoffTs(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackLiftoffMaxAccTs(long long* liftoff_max_acc_ts, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_LIFTOFF_MAX_ACC_TS + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_LIFTOFF_MAX_ACC_TS, &bits);
        
        if(success)
        {
            *liftoff_max_acc_ts = LowRateTMConversion::bitsToLiftoffMaxAccTs(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackLiftoffMaxAcc(float* liftoff_max_acc, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_LIFTOFF_MAX_ACC + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_LIFTOFF_MAX_ACC, &bits);
        
        if(success)
        {
            *liftoff_max_acc = LowRateTMConversion::bitsToLiftoffMaxAcc(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackMaxZspeedTs(long long* max_zspeed_ts, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_MAX_ZSPEED_TS + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_MAX_ZSPEED_TS, &bits);
        
        if(success)
        {
            *max_zspeed_ts = LowRateTMConversion::bitsToMaxZspeedTs(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackMaxZspeed(float* max_zspeed, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_MAX_ZSPEED + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_MAX_ZSPEED, &bits);
        
        if(success)
        {
            *max_zspeed = LowRateTMConversion::bitsToMaxZspeed(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackMaxSpeedAltitude(float* max_speed_altitude, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_MAX_SPEED_ALTITUDE + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_MAX_SPEED_ALTITUDE, &bits);
        
        if(success)
        {
            *max_speed_altitude = LowRateTMConversion::bitsToMaxSpeedAltitude(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackApogeeTs(long long* apogee_ts, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_APOGEE_TS + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_APOGEE_TS, &bits);
        
        if(success)
        {
            *apogee_ts = LowRateTMConversion::bitsToApogeeTs(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackNxpMinPressure(float* nxp_min_pressure, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_NXP_MIN_PRESSURE + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_NXP_MIN_PRESSURE, &bits);
        
        if(success)
        {
            *nxp_min_pressure = LowRateTMConversion::bitsToNxpMinPressure(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackHwMinPressure(float* hw_min_pressure, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_HW_MIN_PRESSURE + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_HW_MIN_PRESSURE, &bits);
        
        if(success)
        {
            *hw_min_pressure = LowRateTMConversion::bitsToHwMinPressure(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackKalmanMinPressure(float* kalman_min_pressure, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_KALMAN_MIN_PRESSURE + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_KALMAN_MIN_PRESSURE, &bits);
        
        if(success)
        {
            *kalman_min_pressure = LowRateTMConversion::bitsToKalmanMinPressure(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackDigitalMinPressure(float* digital_min_pressure, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_DIGITAL_MIN_PRESSURE + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_DIGITAL_MIN_PRESSURE, &bits);
        
        if(success)
        {
            *digital_min_pressure = LowRateTMConversion::bitsToDigitalMinPressure(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackBaroMaxAltitutde(float* baro_max_altitutde , size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_BARO_MAX_ALTITUTDE  + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_BARO_MAX_ALTITUTDE , &bits);
        
        if(success)
        {
            *baro_max_altitutde  = LowRateTMConversion::bitsToBaroMaxAltitutde(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackGpsMaxAltitude(float* gps_max_altitude, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_GPS_MAX_ALTITUDE + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_GPS_MAX_ALTITUDE, &bits);
        
        if(success)
        {
            *gps_max_altitude = LowRateTMConversion::bitsToGpsMaxAltitude(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackApogeeLat(float* apogee_lat, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_APOGEE_LAT + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_APOGEE_LAT, &bits);
        
        if(success)
        {
            *apogee_lat = LowRateTMConversion::bitsToApogeeLat(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackApogeeLon(float* apogee_lon, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_APOGEE_LON + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_APOGEE_LON, &bits);
        
        if(success)
        {
            *apogee_lon = LowRateTMConversion::bitsToApogeeLon(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackDrogueDplTs(long long* drogue_dpl_ts, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_DROGUE_DPL_TS + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_DROGUE_DPL_TS, &bits);
        
        if(success)
        {
            *drogue_dpl_ts = LowRateTMConversion::bitsToDrogueDplTs(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackDrogueDplMaxAcc(float* drogue_dpl_max_acc, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_DROGUE_DPL_MAX_ACC + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_DROGUE_DPL_MAX_ACC, &bits);
        
        if(success)
        {
            *drogue_dpl_max_acc = LowRateTMConversion::bitsToDrogueDplMaxAcc(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackMainDplTs(long long* main_dpl_ts, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_MAIN_DPL_TS + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_MAIN_DPL_TS, &bits);
        
        if(success)
        {
            *main_dpl_ts = LowRateTMConversion::bitsToMainDplTs(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackMainDplAltitude(float* main_dpl_altitude, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_MAIN_DPL_ALTITUDE + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_MAIN_DPL_ALTITUDE, &bits);
        
        if(success)
        {
            *main_dpl_altitude = LowRateTMConversion::bitsToMainDplAltitude(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackMainDplZspeed(float* main_dpl_zspeed, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_MAIN_DPL_ZSPEED + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_MAIN_DPL_ZSPEED, &bits);
        
        if(success)
        {
            *main_dpl_zspeed = LowRateTMConversion::bitsToMainDplZspeed(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackMainDplAcc(float* main_dpl_acc, size_t packet_index)
    {
        if(packet_index >= 1)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_MAIN_DPL_ACC + packet_index * LR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_MAIN_DPL_ACC, &bits);
        
        if(success)
        {
            *main_dpl_acc = LowRateTMConversion::bitsToMainDplAcc(bits);
            return true;
        }else
        {
            return false;
        }

    }


private:
    enum FieldIndex
    {
        INDEX_LIFTOFF_TS = 0,
        INDEX_LIFTOFF_MAX_ACC_TS = 25,
        INDEX_LIFTOFF_MAX_ACC = 50,
        INDEX_MAX_ZSPEED_TS = 61,
        INDEX_MAX_ZSPEED = 86,
        INDEX_MAX_SPEED_ALTITUDE = 96,
        INDEX_APOGEE_TS = 108,
        INDEX_NXP_MIN_PRESSURE = 133,
        INDEX_HW_MIN_PRESSURE = 145,
        INDEX_KALMAN_MIN_PRESSURE = 157,
        INDEX_DIGITAL_MIN_PRESSURE = 169,
        INDEX_BARO_MAX_ALTITUTDE  = 182,
        INDEX_GPS_MAX_ALTITUDE = 194,
        INDEX_APOGEE_LAT = 204,
        INDEX_APOGEE_LON = 214,
        INDEX_DROGUE_DPL_TS = 224,
        INDEX_DROGUE_DPL_MAX_ACC = 249,
        INDEX_MAIN_DPL_TS = 260,
        INDEX_MAIN_DPL_ALTITUDE = 285,
        INDEX_MAIN_DPL_ZSPEED = 298,
        INDEX_MAIN_DPL_ACC = 308
    };

    enum FieldSize
    {
        SIZE_LIFTOFF_TS = 25,
        SIZE_LIFTOFF_MAX_ACC_TS = 25,
        SIZE_LIFTOFF_MAX_ACC = 11,
        SIZE_MAX_ZSPEED_TS = 25,
        SIZE_MAX_ZSPEED = 10,
        SIZE_MAX_SPEED_ALTITUDE = 12,
        SIZE_APOGEE_TS = 25,
        SIZE_NXP_MIN_PRESSURE = 12,
        SIZE_HW_MIN_PRESSURE = 12,
        SIZE_KALMAN_MIN_PRESSURE = 12,
        SIZE_DIGITAL_MIN_PRESSURE = 13,
        SIZE_BARO_MAX_ALTITUTDE  = 12,
        SIZE_GPS_MAX_ALTITUDE = 10,
        SIZE_APOGEE_LAT = 10,
        SIZE_APOGEE_LON = 10,
        SIZE_DROGUE_DPL_TS = 25,
        SIZE_DROGUE_DPL_MAX_ACC = 11,
        SIZE_MAIN_DPL_TS = 25,
        SIZE_MAIN_DPL_ALTITUDE = 13,
        SIZE_MAIN_DPL_ZSPEED = 10,
        SIZE_MAIN_DPL_ACC = 11
    };

    uint8_t *packet;
    BitPacker bitpacker;
};