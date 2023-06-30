/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Giulia Ghirardini
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

namespace LPS22DFDefs
{

static constexpr uint32_t WHO_AM_I_VALUE = 0xb4;

static constexpr float TEMP_SENS = 100;    ///< [LSB/°C]
static constexpr float PRES_SENS = 40.96;  ///< [LSB/Pa]

enum Registers : uint8_t
{
    INTERRUPT_CFG = 0x0b,  ///< Interrupt mode for pressure acquisition

    THS_P_L = 0x0c,  ///< User-defined threshold LSB register
    THS_P_H = 0x0d,  ///< User-defined threshold MSB register

    IF_CTRL = 0x0e,  ///< Interface control register

    WHO_AM_I = 0x0f,  ///< Device Who am I register

    CTRL_REG1 = 0x10,  ///< Control Register 1 [ODR, AVG]
    CTRL_REG2 = 0x11,  ///< Control Register 2
    CTRL_REG3 = 0x12,  ///< Control Register 3
    CTRL_REG4 = 0x13,  ///< Control Register 4

    FIFO_CTRL = 0x14,  ///< FIFO control register
    FIFO_WTM  = 0x15,  ///< FIFO threshold setting register

    REF_P_L = 0x16,  ///< Reference pressure LSB data
    REF_P_H = 0x17,  ///< Reference pressure MSB data

    FIFO_STATUS1 = 0x25,  ///< FIFO status register 1
    FIFO_STATUS2 = 0x26,  ///< FIFO status register 2

    STATUS = 0x27,  ///< Status register

    PRESS_OUT_XL = 0x28,  ///< Pressure output value LSB data
    PRESS_OUT_L  = 0x29,  ///< Pressure output value middle data
    PRESS_OUT_H  = 0x2a,  ///< Pressure output value MSB data

    TEMP_OUT_L = 0x2b,  ///< Temperature output value LSB data
    TEMP_OUT_H = 0x2c,  ///< Temperature output value MSB data

    FIFO_DATA_OUT_PRESS_XL = 0x78,  ///< FIFO pressure output LSB data
    FIFO_DATA_OUT_PRESS_L  = 0x79,  ///< FIFO pressure output middle data
    FIFO_DATA_OUT_PRESS_H  = 0x7a,  ///< FIFO pressure output MSB data
};

enum IF_CTRL : uint8_t
{
    CS_PU_DIS   = 1 << 1,
    INT_PD_DIS  = 1 << 2,
    SDO_PU_EN   = 1 << 3,
    SDA_PU_EN   = 1 << 4,
    SIM         = 1 << 5,
    I2C_I3C_DIS = 1 << 6,  ///< Disable I2C and I3C digital interfaces
    INT_EN_I3C  = 1 << 7
};

enum CTRL_REG2 : uint8_t
{
    ONE_SHOT_START = 1 << 0,  ///< Enable one-shot mode
    SWRESET        = 1 << 2,  ///< Software reset
    BDU            = 1 << 3,  ///< Block data update
    EN_LPFP        = 1 << 4,  ///< Enable low-pass filter on pressure data
    LFPF_CFG       = 1 << 5,  ///< Low-pass filter configuration
    FS_MODE        = 1 << 6,  ///< Full-scale selection
    BOOT           = 1 << 7   ///< Reboot memory content
};

enum CTRL_REG3 : uint8_t
{
    IF_ADD_INC = 1 << 0,  ///< Increment register during a multiple byte access
    PP_OD      = 1 << 1,  ///< Push-pull/open-drain selection on interrupt pin
    INT_H_L    = 1 << 3   ///< Select interrupt active-high, active-low
};

enum CTRL_REG4 : uint8_t
{
    INT_F_OVR  = 1 << 0,  ///< FIFO overrun status on INT_DRDY pin
    INT_F_WTM  = 1 << 1,  ///< FIFO threshold status on INT_DRDY pin
    INT_F_FULL = 1 << 2,  ///< FIFO full flag on INT_DRDY pin
    INT_EN     = 1 << 4,  ///< Interrupt signal on INT_DRDY pin
    DRDY       = 1 << 5,  ///< Date-ready signal on INT_DRDY pin
    DRDY_PLS   = 1 << 6   ///< Data-ready pulsed on INT_DRDY pin
};

enum STATUS : uint8_t
{
    P_DA = 1 << 0,  ///< Pressure data available
    T_DA = 1 << 1,  ///< Temperature data available
    P_OR = 1 << 4,  ///< Pressure data overrun
    T_OR = 1 << 5   ///< Temperature data overrun
};

}  // namespace LPS22DFDefs

}  // namespace Boardcore