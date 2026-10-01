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

#pragma once

#include <stdint.h>

#include "SCH16TK10Defs.h"

namespace Boardcore
{
namespace SCH16TK10Frames
{

using SCH16TK10Defs::Register;
using SCH16TK10Defs::SensorStatus;
using SCH16TK10Defs::SLAVE_ID;
/**
 * Helper struct to handle 32-bit SPI communication for this sensor
 */
struct SPIFrame32
{
    using raw_t                        = uint32_t;
    static constexpr unsigned int bits = 32;
    static constexpr uint8_t frameType = 0;  // 32 bits SPI Frames

    /**
     * Response frame sent by the sensor
     */
    struct Response
    {
        bool condition;  // 1 = Read Acceleration, Gyro or Temperature; 0 =
                         // Read from every other register
        uint8_t sourceAddress;
        SensorStatus sensorStatus;
        uint16_t data;
        bool checkCRC;
    };

    /**
     * @brief Evaluate Cyclic redundancy check from the SPI Frame (For 32 bits
     * frames - from the datasheet)
     * @param SPIFrame Frame received from the sensor
     */
    static constexpr uint8_t calculateCRC(raw_t SPIFrame)
    {
        uint32_t data = SPIFrame & 0xFFFFFFF8;
        uint8_t crc   = 0x05;

        for (int i = 31; i >= 0; i--)
        {
            uint8_t data_bit = (data >> i) & 0x01;
            crc = crc & 0x4 ? (uint8_t)((crc << 1) ^ 0x3) ^ data_bit
                            : (uint8_t)(crc << 1) | data_bit;
            crc &= 0x7;
        }
        return crc;
    }

    /**
     * @brief Encode 32-bit SPI Frame
     * @param regAddr Target register address
     * @param mode Read = 0, Write = 1
     * @param data Data to send to the sensor
     */
    static constexpr raw_t encode(Register regAddr, bool mode, uint32_t data)
    {
        uint32_t frame = 0;

        raw_t ta = ((static_cast<raw_t>(SLAVE_ID) & 0x3) << 8) |
                   (static_cast<raw_t>(regAddr) & 0xFF);

        frame |= (ta << 22);
        frame |= (static_cast<raw_t>(mode) << 21);
        frame |= (static_cast<raw_t>(frameType) << 19);
        frame |= ((static_cast<raw_t>(data) & 0xFFFF) << 3);
        frame |= calculateCRC(frame);

        return frame;
    }

    /**
     * @brief Decode 32-bit SPI Frame
     * @param frame SPI frame received from the sensor to decode
     */
    static constexpr Response decode(uint32_t frame)
    {
        Response resp      = {};
        resp.condition     = (frame >> 31) & 0x1;
        resp.sourceAddress = static_cast<uint8_t>((frame >> 21) & 0xFF);
        resp.sensorStatus  = static_cast<SensorStatus>(
            (((frame >> 20) & 0x1) << 1) | ((frame >> 3) & 0x1));
        resp.data = static_cast<uint16_t>((frame >> 4) & 0xFFFF);

        if (calculateCRC(frame) == (frame & 0x7))
            resp.checkCRC = true;
        else
            resp.checkCRC = false;

        return resp;
    }
};

/**
 * Helper struct to handle 48-bit SPI communication for this sensor
 */
struct SPIFrame48
{
    using raw_t                        = uint64_t;
    static constexpr unsigned int bits = 48;
    static constexpr uint8_t frameType = 1;  // 48 bits SPI Frames

    /**
     * Response frame sent by the sensor
     */
    struct Response
    {
        bool condition;  // 1 = Read Acceleration, Gyro or Temperature; 0 =
                         // Read from every other register
        uint8_t sourceAddress;
        bool ids;  // Internal data status
        bool commandError;
        uint8_t dcnt;
        uint32_t data;
        SensorStatus sensorStatus;
        bool checkCRC;
    };

    /**
     * @brief Evaluate Cyclic redundancy check from the SPI Frame (For 48 bits
     * frames - from the datasheet)
     * @param SPIFrame Frame received from the sensor
     */
    static constexpr uint8_t calculateCRC(raw_t SPIFrame)
    {
        uint64_t data = SPIFrame & 0xFFFFFFFFFF00LL;
        uint8_t crc   = 0xFF;

        for (int i = 47; i >= 0; i--)
        {
            uint8_t data_bit = (data >> i) & 0x01;
            crc = crc & 0x80 ? (uint8_t)((crc << 1) ^ 0x2F) ^ data_bit
                             : (uint8_t)(crc << 1) | data_bit;
        }
        return crc;
    }

    /**
     * @brief Encode 48-bit SPI Frame
     * @param regAddr Target register address
     * @param mode Read = 0, Write = 1
     * @param data Data to send to the sensor
     */
    static constexpr raw_t encode(Register regAddr, bool mode, uint32_t data)
    {
        raw_t frame = 0;

        raw_t ta = ((static_cast<raw_t>(SLAVE_ID) & 0x3) << 8) |
                   (static_cast<raw_t>(regAddr) & 0xFF);

        frame |= (ta << 38);
        frame |= (static_cast<raw_t>(mode) << 37);
        frame |= (static_cast<raw_t>(frameType) << 35);
        frame |= ((static_cast<raw_t>(data) & 0xFFFFF) << 8);
        frame |= calculateCRC(frame);

        return frame;
    }

    /**
     * @brief Decode 48-bit SPI Frame
     * @param frame SPI frame received from the sensor to decode
     */
    static constexpr Response decode(uint64_t frame)
    {
        Response resp      = {};
        resp.condition     = ((frame >> 47) & 0x1);
        resp.sourceAddress = static_cast<uint8_t>((frame >> 37) & 0xFF);
        resp.ids           = ((frame >> 36) & 0x1);
        resp.commandError  = ((frame >> 35) & 0x1);
        resp.dcnt          = static_cast<uint8_t>((frame >> 29) & 0xF);
        resp.data          = static_cast<uint32_t>((frame >> 8) & 0xFFFFF);
        resp.sensorStatus  = static_cast<SensorStatus>((frame >> 33) & 0x3);

        if (calculateCRC(frame) == (frame & 0xFF))
            resp.checkCRC = true;
        else
            resp.checkCRC = false;

        return resp;
    }
};

}  // namespace SCH16TK10Frames
}  // namespace Boardcore
