/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Aldas Lenkšas
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 copy
 * of this software and associated documentation files (the "Software"), to
 deal
 * in the Software without restriction, including without limitation the
 rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#pragma once

#include <units/Time.h>

namespace Boardcore
{

namespace AS5047UDefs
{

/**
 * @brief Addresses of the sensor registers.
 */
enum class Registers : uint16_t
{
    // Volatile Registers
    NOP          = 0x0000,
    ERRFL        = 0x0001,
    PROG         = 0x0003,
    DIA          = 0x3FF5,
    AGC          = 0x3FF9,
    SIN_DATA     = 0x3FFA,
    COS_DATA     = 0x3FFB,
    VEL          = 0x3FFC,
    MAG          = 0x3FFD,
    ANGLEUNC     = 0x3FFE,
    ANGLECOM     = 0x3FFF,
    ECC_CHECKSUM = 0x00D1,

    // Non-Volatile Registers (OTP)
    DISABLE   = 0x0015,
    ZPOSM     = 0x0016,
    ZPOSL     = 0x0017,
    SETTINGS1 = 0x0018,
    SETTINGS2 = 0x0019,
    SETTINGS3 = 0x001A,
    ECC       = 0x001B,
};

enum class Mode : uint8_t
{
    WRITE = 0,
    READ  = 1
};

/**@brief Whether to use UVW.
 * @note Currently UVW::ON is not supported.
 */
enum class UVW : uint8_t
{
    ON  = 0b0,
    OFF = 0b1,
};

/**@brief Whether to use ABI.
 * @note Currently ABI::ON is not supported.
 */
enum class ABI : uint8_t
{
    ON  = 0b0,
    OFF = 0b1,
};

// TODO support Filter::ON
/**@brief Whether to use ABI.
 * @note Currently Filter::ON is not supported.
 */
enum class Filter : uint8_t
{
    ON  = 0b0,
    OFF = 0b1,
};

enum class FilterK : uint8_t
{
    K_0 = 0b000,
    K_1 = 0b001,
    K_2 = 0b010,
    K_3 = 0b011,
    K_4 = 0b100,
    K_5 = 0b101,
    K_6 = 0b110,
};

enum class NoiseSet : uint8_t
{
    FALSE = 0b0,
    TRUE  = 0b1,
};

// TODO Support ON for DAECompensation.
/**
 * @brief Whether Dynamic Angle Error (DAE) compensation is on.
 * @note ON is not supported.
 */
enum class DAECompensation : uint8_t
{
    ON  = 0b0,
    OFF = 0b1,
};

/**
 * @brief Select which data will be read with sample().
 * @note ANGLECOM uses DAE for angle error compensation, ANGLEUNC reads the raw
 * uncompensated angle.
 */
enum class DAECStatus : uint8_t
{
    ON  = 0b0,
    OFF = 0b1,
};

/**
 * @brief Whether to enable PWM (Pulse Width Modulation).
 * @note PWM::ON is not supported.
 */
enum class PWM : uint8_t
{
    OFF = 0b0,
    ON  = 0b1,
};

/**
 * @brief Resolution of the sensor reading.
 * @note Sensor is 14-bit, so 360 degrees are mapped into 2^14 values,
 * one bit has resolution of 360/(2^14).
 */
static constexpr float SPI_ANGLE_RES = 0.02197265625f;

}  // namespace AS5047UDefs

}  // namespace Boardcore
