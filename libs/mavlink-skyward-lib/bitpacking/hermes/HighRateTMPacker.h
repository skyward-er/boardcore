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

class HighRateTMPacker
{
public:
    static constexpr int HR_TM_PACKET_SIZE            = 104;
    static constexpr int HR_TM_SINGLE_PACKET_SIZE_BITS = 208;

    static_assert(MAVLINK_MSG_HR_TM_FIELD_PAYLOAD_LEN == HR_TM_PACKET_SIZE,
                  "Payload size mismatch! Mavlnk payload size differes from declared size. Maybe your mavlink definitions are outdated?");




    HighRateTMPacker(uint8_t *packet) : packet(packet), bitpacker(packet, HR_TM_PACKET_SIZE)
    {
    }

    
    bool packTimestamp(long long timestamp, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::timestampToBits(timestamp);

        return bitpacker.pack(
            INDEX_TIMESTAMP + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_TIMESTAMP, bits);
    }

    bool packPressureAda(float pressure_ada, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::pressureAdaToBits(pressure_ada);

        return bitpacker.pack(
            INDEX_PRESSURE_ADA + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_PRESSURE_ADA, bits);
    }

    bool packPressureDigi(float pressure_digi, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::pressureDigiToBits(pressure_digi);

        return bitpacker.pack(
            INDEX_PRESSURE_DIGI + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_PRESSURE_DIGI, bits);
    }

    bool packMslAltitude(float msl_altitude, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::mslAltitudeToBits(msl_altitude);

        return bitpacker.pack(
            INDEX_MSL_ALTITUDE + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_MSL_ALTITUDE, bits);
    }

    bool packAglAltitude(float agl_altitude, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::aglAltitudeToBits(agl_altitude);

        return bitpacker.pack(
            INDEX_AGL_ALTITUDE + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_AGL_ALTITUDE, bits);
    }

    bool packVertSpeed(float vert_speed, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::vertSpeedToBits(vert_speed);

        return bitpacker.pack(
            INDEX_VERT_SPEED + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_VERT_SPEED, bits);
    }

    bool packVertSpeed2(float vert_speed_2, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::vertSpeed2ToBits(vert_speed_2);

        return bitpacker.pack(
            INDEX_VERT_SPEED_2 + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_VERT_SPEED_2, bits);
    }

    bool packAccX(float acc_x, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::accXToBits(acc_x);

        return bitpacker.pack(
            INDEX_ACC_X + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_ACC_X, bits);
    }

    bool packAccY(float acc_y, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::accYToBits(acc_y);

        return bitpacker.pack(
            INDEX_ACC_Y + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_ACC_Y, bits);
    }

    bool packAccZ(float acc_z, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::accZToBits(acc_z);

        return bitpacker.pack(
            INDEX_ACC_Z + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_ACC_Z, bits);
    }

    bool packGyroX(float gyro_x, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::gyroXToBits(gyro_x);

        return bitpacker.pack(
            INDEX_GYRO_X + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_GYRO_X, bits);
    }

    bool packGyroY(float gyro_y, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::gyroYToBits(gyro_y);

        return bitpacker.pack(
            INDEX_GYRO_Y + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_GYRO_Y, bits);
    }

    bool packGyroZ(float gyro_z, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::gyroZToBits(gyro_z);

        return bitpacker.pack(
            INDEX_GYRO_Z + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_GYRO_Z, bits);
    }

    bool packGpsLat(double gps_lat, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::gpsLatToBits(gps_lat);

        return bitpacker.pack(
            INDEX_GPS_LAT + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_GPS_LAT, bits);
    }

    bool packGpsLon(double gps_lon, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::gpsLonToBits(gps_lon);

        return bitpacker.pack(
            INDEX_GPS_LON + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_GPS_LON, bits);
    }

    bool packGpsAlt(float gps_alt, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::gpsAltToBits(gps_alt);

        return bitpacker.pack(
            INDEX_GPS_ALT + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_GPS_ALT, bits);
    }

    bool packTemperature(float temperature, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::temperatureToBits(temperature);

        return bitpacker.pack(
            INDEX_TEMPERATURE + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_TEMPERATURE, bits);
    }

    bool packFmmState(uint8_t fmm_state, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::fmmStateToBits(fmm_state);

        return bitpacker.pack(
            INDEX_FMM_STATE + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_FMM_STATE, bits);
    }

    bool packDplState(uint8_t dpl_state, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::dplStateToBits(dpl_state);

        return bitpacker.pack(
            INDEX_DPL_STATE + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_DPL_STATE, bits);
    }

    bool packPinLaunch(uint8_t pin_launch, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::pinLaunchToBits(pin_launch);

        return bitpacker.pack(
            INDEX_PIN_LAUNCH + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_PIN_LAUNCH, bits);
    }

    bool packPinNosecone(uint8_t pin_nosecone, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::pinNoseconeToBits(pin_nosecone);

        return bitpacker.pack(
            INDEX_PIN_NOSECONE + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_PIN_NOSECONE, bits);
    }

    bool packGpsFix(uint8_t gps_fix, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits = HighRateTMConversion::gpsFixToBits(gps_fix);

        return bitpacker.pack(
            INDEX_GPS_FIX + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_GPS_FIX, bits);
    }


    
    bool unpackTimestamp(long long* timestamp, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_TIMESTAMP + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_TIMESTAMP, &bits);
        
        if(success)
        {
            *timestamp = HighRateTMConversion::bitsToTimestamp(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackPressureAda(float* pressure_ada, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_PRESSURE_ADA + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_PRESSURE_ADA, &bits);
        
        if(success)
        {
            *pressure_ada = HighRateTMConversion::bitsToPressureAda(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackPressureDigi(float* pressure_digi, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_PRESSURE_DIGI + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_PRESSURE_DIGI, &bits);
        
        if(success)
        {
            *pressure_digi = HighRateTMConversion::bitsToPressureDigi(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackMslAltitude(float* msl_altitude, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_MSL_ALTITUDE + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_MSL_ALTITUDE, &bits);
        
        if(success)
        {
            *msl_altitude = HighRateTMConversion::bitsToMslAltitude(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackAglAltitude(float* agl_altitude, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_AGL_ALTITUDE + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_AGL_ALTITUDE, &bits);
        
        if(success)
        {
            *agl_altitude = HighRateTMConversion::bitsToAglAltitude(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackVertSpeed(float* vert_speed, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_VERT_SPEED + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_VERT_SPEED, &bits);
        
        if(success)
        {
            *vert_speed = HighRateTMConversion::bitsToVertSpeed(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackVertSpeed2(float* vert_speed_2, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_VERT_SPEED_2 + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_VERT_SPEED_2, &bits);
        
        if(success)
        {
            *vert_speed_2 = HighRateTMConversion::bitsToVertSpeed2(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackAccX(float* acc_x, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_ACC_X + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_ACC_X, &bits);
        
        if(success)
        {
            *acc_x = HighRateTMConversion::bitsToAccX(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackAccY(float* acc_y, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_ACC_Y + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_ACC_Y, &bits);
        
        if(success)
        {
            *acc_y = HighRateTMConversion::bitsToAccY(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackAccZ(float* acc_z, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_ACC_Z + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_ACC_Z, &bits);
        
        if(success)
        {
            *acc_z = HighRateTMConversion::bitsToAccZ(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackGyroX(float* gyro_x, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_GYRO_X + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_GYRO_X, &bits);
        
        if(success)
        {
            *gyro_x = HighRateTMConversion::bitsToGyroX(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackGyroY(float* gyro_y, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_GYRO_Y + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_GYRO_Y, &bits);
        
        if(success)
        {
            *gyro_y = HighRateTMConversion::bitsToGyroY(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackGyroZ(float* gyro_z, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_GYRO_Z + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_GYRO_Z, &bits);
        
        if(success)
        {
            *gyro_z = HighRateTMConversion::bitsToGyroZ(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackGpsLat(double* gps_lat, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_GPS_LAT + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_GPS_LAT, &bits);
        
        if(success)
        {
            *gps_lat = HighRateTMConversion::bitsToGpsLat(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackGpsLon(double* gps_lon, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_GPS_LON + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_GPS_LON, &bits);
        
        if(success)
        {
            *gps_lon = HighRateTMConversion::bitsToGpsLon(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackGpsAlt(float* gps_alt, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_GPS_ALT + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_GPS_ALT, &bits);
        
        if(success)
        {
            *gps_alt = HighRateTMConversion::bitsToGpsAlt(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackTemperature(float* temperature, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_TEMPERATURE + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_TEMPERATURE, &bits);
        
        if(success)
        {
            *temperature = HighRateTMConversion::bitsToTemperature(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackFmmState(uint8_t* fmm_state, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_FMM_STATE + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_FMM_STATE, &bits);
        
        if(success)
        {
            *fmm_state = HighRateTMConversion::bitsToFmmState(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackDplState(uint8_t* dpl_state, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_DPL_STATE + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_DPL_STATE, &bits);
        
        if(success)
        {
            *dpl_state = HighRateTMConversion::bitsToDplState(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackPinLaunch(uint8_t* pin_launch, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_PIN_LAUNCH + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_PIN_LAUNCH, &bits);
        
        if(success)
        {
            *pin_launch = HighRateTMConversion::bitsToPinLaunch(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackPinNosecone(uint8_t* pin_nosecone, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_PIN_NOSECONE + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_PIN_NOSECONE, &bits);
        
        if(success)
        {
            *pin_nosecone = HighRateTMConversion::bitsToPinNosecone(bits);
            return true;
        }else
        {
            return false;
        }

    }

    bool unpackGpsFix(uint8_t* gps_fix, size_t packet_index)
    {
        if(packet_index >= 4)
        {
            return false;
        }

        // Convert data to a suitable format and store in an unsigned int
        uint64_t bits;

        bool success = bitpacker.unpack(
            INDEX_GPS_FIX + packet_index * HR_TM_SINGLE_PACKET_SIZE_BITS,
            SIZE_GPS_FIX, &bits);
        
        if(success)
        {
            *gps_fix = HighRateTMConversion::bitsToGpsFix(bits);
            return true;
        }else
        {
            return false;
        }

    }


private:
    enum FieldIndex
    {
        INDEX_TIMESTAMP = 0,
        INDEX_PRESSURE_ADA = 25,
        INDEX_PRESSURE_DIGI = 37,
        INDEX_MSL_ALTITUDE = 50,
        INDEX_AGL_ALTITUDE = 62,
        INDEX_VERT_SPEED = 71,
        INDEX_VERT_SPEED_2 = 81,
        INDEX_ACC_X = 91,
        INDEX_ACC_Y = 102,
        INDEX_ACC_Z = 113,
        INDEX_GYRO_X = 124,
        INDEX_GYRO_Y = 135,
        INDEX_GYRO_Z = 146,
        INDEX_GPS_LAT = 157,
        INDEX_GPS_LON = 168,
        INDEX_GPS_ALT = 179,
        INDEX_TEMPERATURE = 189,
        INDEX_FMM_STATE = 196,
        INDEX_DPL_STATE = 201,
        INDEX_PIN_LAUNCH = 205,
        INDEX_PIN_NOSECONE = 206,
        INDEX_GPS_FIX = 207
    };

    enum FieldSize
    {
        SIZE_TIMESTAMP = 25,
        SIZE_PRESSURE_ADA = 12,
        SIZE_PRESSURE_DIGI = 13,
        SIZE_MSL_ALTITUDE = 12,
        SIZE_AGL_ALTITUDE = 9,
        SIZE_VERT_SPEED = 10,
        SIZE_VERT_SPEED_2 = 10,
        SIZE_ACC_X = 11,
        SIZE_ACC_Y = 11,
        SIZE_ACC_Z = 11,
        SIZE_GYRO_X = 11,
        SIZE_GYRO_Y = 11,
        SIZE_GYRO_Z = 11,
        SIZE_GPS_LAT = 11,
        SIZE_GPS_LON = 11,
        SIZE_GPS_ALT = 10,
        SIZE_TEMPERATURE = 7,
        SIZE_FMM_STATE = 5,
        SIZE_DPL_STATE = 4,
        SIZE_PIN_LAUNCH = 1,
        SIZE_PIN_NOSECONE = 1,
        SIZE_GPS_FIX = 1
    };

    uint8_t *packet;
    BitPacker bitpacker;
};