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

namespace Boardcore
{
namespace SCH16TK10Defs
{

// Registers of the Sensor
enum class Register : uint8_t
{
    // Bank 0
    RATE_X1 = 0x0001,
    RATE_Y1 = 0x0002,
    RATE_Z1 = 0x0003,
    ACC_X1  = 0x0004,
    ACC_Y1  = 0x0005,
    ACC_Z1  = 0x0006,
    ACC_X3  = 0x0007,
    ACC_Y3  = 0x0008,
    ACC_Z3  = 0x0009,
    RATE_X2 = 0x000A,
    RATE_Y2 = 0x000B,
    RATE_Z2 = 0x000C,
    ACC_X2  = 0x000D,
    ACC_Y2  = 0x000E,
    ACC_Z2  = 0x000F,

    // Bank 1
    TEMP             = 0x0010,
    RATE_DCNT        = 0x0011,
    ACC_DCNT         = 0x0012,
    FREQ_CNTR        = 0x0013,
    STAT_SUM         = 0x0014,
    STAT_SUM_SAT     = 0x0015,
    STAT_COM         = 0x0016,
    STAT_RATE_COM    = 0x0017,
    STAT_RATE_X      = 0x0018,
    STAT_RATE_Y      = 0x0019,
    STAT_RATE_Z      = 0x001A,
    STAT_ACC_X       = 0x001B,
    STAT_ACC_Y       = 0x001C,
    STAT_ACC_Z       = 0x001D,
    STAT_SYNC_ACTIVE = 0x001E,
    STAT_INFO        = 0x001F,

    // Bank 2
    CTRL_FILT_RATE  = 0x0025,
    CTRL_FILT_ACC12 = 0x0026,
    CTRL_FILT_ACC3  = 0x0027,
    CTRL_RATE       = 0x0028,
    CTRL_ACC12      = 0x0029,
    CTRL_ACC3       = 0x002A,

    // Bank 3
    CTRL_USER_IF = 0x0033,
    CTRL_ST      = 0x0034,
    CTRL_MODE    = 0x0035,
    CTRL_RESET   = 0x0036,
    SYS_TEST     = 0x0037,
    SPARE_1      = 0x0038,
    SPARE_2      = 0x0039,
    SPARE_3      = 0x003A,
    ASIC_ID      = 0x003B,
    COMP_ID      = 0x003C,
    SN_ID1       = 0x003D,
    SN_ID2       = 0x003E,
    SN_ID3       = 0x003F
};

enum class SensorStatus : uint8_t
{
    NORMAL           = 0b00,
    ERROR            = 0b01,
    SATURATION_ERROR = 0b10,
    INIT_RUNNING     = 0b11
};

/**
 * The following address enables multi slave addressing on the same SPI Chip
 * select. Since this is not used these two values are dropped to 0 (Check if
 * compliant with schematic) - If needed for next year consider putting it in
 * the class constructor
 */
static constexpr uint8_t SLAVE_ID = 0;

static constexpr unsigned int WHOAMI = 0;  // TBD

}  // namespace SCH16TK10Defs
}  // namespace Boardcore
