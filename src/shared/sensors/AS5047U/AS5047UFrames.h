/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Aldas Lenkšas
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

#include <concepts>

#include "AS5047UDefs.h"

namespace Boardcore
{
namespace AS5047UFrames
{

using namespace AS5047UDefs;

/**
 * @brief Helper struct to handle 16-bit SPI communication for this sensor
 */
struct SPIFrame16
{
    using raw_t                        = uint16_t;
    static constexpr unsigned int bits = 16;
    static constexpr uint8_t frameType = 0;  // 32 bits SPI Frames

    /**
     * Response frame sent by the sensor
     */
    struct Response
    {
        bool warning;
        bool error;
        uint16_t data;
        bool okCRC = true;
    };

    /**
     * @brief Encode 16-bit SPI Frame
     * @param regAddr Target register address to read
     */
    static constexpr raw_t encode(Registers regAddr)
    {
        raw_t frame = 0;

        frame |= (0b1 << 14);  // read
        frame |= static_cast<raw_t>(regAddr);

        return frame;
    }

    /**
     * @brief Decode 16-bit SPI Frame
     * @param frame SPI frame received from the sensor to decode
     */
    static constexpr Response decode(raw_t frame)
    {
        Response resp = {};
        resp.warning  = (frame >> 15) & 0b1;
        resp.error    = (frame >> 14) & 0b1;
        resp.data     = frame & 0x3FFF;
        return resp;
    }
};

/**
 * Helper struct to handle 24-bit SPI communication for this sensor.
 */
struct SPIFrame24
{
    using raw_t                        = uint32_t;
    static constexpr unsigned int bits = 24;
    static constexpr uint8_t frameType = 1;  // 24 bits SPI Frames

    /**
     * Response frame sent by the sensor
     */
    struct Response
    {
        bool warning;
        bool error;
        uint16_t data;
        bool okCRC;
    };

    /**
     * @brief Evaluate Cyclic redundancy check from the SPI Frame (For 24
     bits frames - from the datasheet)
     * @param payload The data to be evaluated
     */
    static constexpr uint8_t calculateCRC(uint16_t data)
    {
        uint8_t crc = 0xC4;

        for (int i = 15; i >= 0; i--)
        {
            uint8_t dataBit = (data >> i) & 0b1;
            uint8_t crcMSB  = (crc >> 7) & 0b1;

            crc <<= 1;

            if (crcMSB ^ dataBit)
                crc ^= 0x1D;
        }

        return crc;
    }

    /**
     * @brief Encode 24-bit SPI Frame
     * @param regAddr Target register address
     * @param mode Read = 0, Write = 1
     * @param data Data to send to the sensor
     */
    static constexpr raw_t encode(Registers regAddr, Mode mode)
    {
        raw_t frame = 0;

        uint8_t crc = calculateCRC(static_cast<uint16_t>(regAddr));

        frame |= (static_cast<raw_t>(mode) << 22);
        frame |= (static_cast<raw_t>(regAddr) << 8);
        frame |= (static_cast<raw_t>(crc));

        return frame;
    }

    /**
     * @brief Decode 24-bit SPI Frame
     * @param frame SPI frame received from the sensor to decode
     */
    static constexpr Response decode(raw_t frame)
    {
        Response resp = {};
        resp.warning  = (frame >> 23) & 0b1;
        resp.error    = (frame >> 22) & 0b1;
        resp.data     = static_cast<uint16_t>((frame >> 8) & 0xFFFF);

        uint8_t crc = static_cast<uint8_t>(frame & 0xFF);
        resp.okCRC  = (crc == calculateCRC(resp.data));

        return resp;
    }
};

template <typename T>
concept Frame = (std::same_as<T, SPIFrame16> || std::same_as<T, SPIFrame24>);

}  // namespace AS5047UFrames
}  // namespace Boardcore
