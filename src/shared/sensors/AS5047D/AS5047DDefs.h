/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Radu Raul
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

#include <units/Time.h>

namespace Boardcore
{
using namespace Units::Time;

namespace AS5047DDefs
{
/**
 * @brief Addresses of the sensor registers
 * @note Even though some registers OTP they
 * can be changed via SPI, but they will act
 * as volatile registers, resetting to the
 * OTP-programmed value.
 */
enum Registers
{
    NOP       = 0x0000,
    ERRFL     = 0x0001,
    PROG      = 0x0003,
    ZPOSM     = 0x0016,
    ZPOSL     = 0x0017,
    SETTINGS1 = 0x0018,
    SETTINGS2 = 0x0019,
    DIAAGC    = 0x3ffc,
    MAG       = 0x3ffd,
    ANGLEUNC  = 0x3ffe,
    ANGLECOM  = 0x3fff,
};

enum ABIResolution : uint8_t
{
    RES_2000 = 0b0000,  /// ABIBIN = 0, ABIRES = 000
    RES_1600 = 0b0001,  /// ABIBIN = 0, ABIRES = 001
    RES_1200 = 0b0010,  /// ABIBIN = 0, ABIRES = 010
    RES_800  = 0b0011,  /// ABIBIN = 0, ABIRES = 011
    RES_400  = 0b0100,  /// ABIBIN = 0, ABIRES = 100
    RES_200  = 0b0101,  /// ABIBIN = 0, ABIRES = 101
    RES_100  = 0b0110,  /// ABIBIN = 0, ABIRES = 110
    RES_32   = 0b0111,  /// ABIBIN = 0, ABIRES = 111
    RES_2048 = 0b1000,  /// ABIBIN = 1, ABIRES = 000
    RES_1024 = 0b1001,  /// ABIBIN = 1, ABIRES = 001
};

enum DataSelect : uint8_t
{
    DAECANG   = 0b0,
    CORDICANG = 0b1
};

enum DAECStatus : uint8_t
{
    DAEC_ON  = 0b0,
    DAEC_OFF = 0b1
};

enum UVWABISelect : uint8_t
{
    ABI = 0b0,
    UVW = 0b1
};

enum PWMSelect : uint8_t
{
    PWM_OFF = 0b0,
    PWM_OFF = 0b1
};

enum ABIRotationDirection : uint8_t
{
    NORMAL   = 0b0,
    INVERSED = 0b1
};

enum UVWPolePairs : uint8_t
{
    ONE   = 0b000,
    TWO   = 0b001,
    THREE = 0b010,
    FOUR  = 0b011,
    FIVE  = 0b100,
    SIX   = 0b101,
    SEVEN = 0b110
};

enum HysteresisConfiguration : uint8_t
{
    CONFIG0 = 0b00,
    CONFIG1 = 0b01,
    CONFIG2 = 0b10,
    CONFIG3 = 0b11
};

static constexpr float SPI_ANGLE_RES              = 360.f / 14.f;
static constexpr uint16_t DAEC_SETTING_MASK       = 0b1111'1111'1110'1111;
static constexpr uint8_t DAEC_SETTING_POS         = 4;
static constexpr uint16_t DATASELECT_SETTING_MASK = 0b1111'1111'1011'1111;
static constexpr uint8_t DATASELECT_SETTING_POS   = 6;

static constexpr Microsecond DELAY_BETWEEN_SPI_TRAN_US = 1_us;
}  // namespace AS5047DDefs

}  // namespace Boardcore
