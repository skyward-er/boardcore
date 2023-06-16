/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

namespace Boardcore
{

/**
 * @brief Various LPS28DFW register/enums definitions.
 */
namespace LPS28DFWDefs
{
static const uint16_t lsp28dfwAddress0{0b1011100};
static const uint16_t lsp28dfwAddress1{0b1011101};

static const uint8_t INTERRUPT_CFG{
    0x0B};  ///< Interrupt mode for pressure acquisition configuration
static const uint8_t THS_P_L{0x0C};   ///< User-defined threshold LSB register
static const uint8_t THS_P_H{0x0D};   ///< User-defined threshold MSB register
static const uint8_t IF_CTRL{0x0E};   ///< Interface control register
static const uint8_t WHO_AM_I{0x0F};  ///< Device Who am I register
static const uint8_t WHO_AM_I_VALUE{0xB4};  ///< Device Who am I value
static const uint8_t CTRL_REG1_addr{0x10};  ///< Control Register 1 [ODR, AVG]

static const uint8_t CTRL_REG2_addr{0x11};  ///< Control Register 2
enum CTRL_REG2 : uint8_t
{
    ONE_SHOT_START = (0b1 << 0),  ///< Enable one-shot mode
    SWRESET        = (0b1 << 2),  ///< Software reset
    BDU            = (0b1 << 3),  ///< Block data update
    EN_LPFP        = (0b1 << 4),  ///< Enable low-pass filter on pressure data
    LFPF_CFG       = (0b1 << 5),  ///< Low-pass filter configuration
    FS_MODE        = (0b1 << 6),  ///< Full-scale selection
    BOOT           = (0b1 << 7)   ///< Reboot memory content
};

static const uint8_t CTRL_REG3_addr{0x12};  ///< Control Register 3
enum CTRL_REG3 : uint8_t
{
    IF_ADD_INC =
        (0b1 << 0),        ///< Increment register during a multiple byte access
    PP_OD   = (0b1 << 1),  ///< Push-pull/open-drain selection on interrupt pin
    INT_H_L = (0b1 << 3)   ///< Select interrupt active-high, active-low
};

static const uint8_t CTRL_REG4_addr{0x13};  ///< Control Register 4
enum CTRL_REG4 : uint8_t
{
    INT_F_OVR  = (0b1 << 0),  ///< FIFO overrun status on INT_DRDY pin
    INT_F_WTM  = (0b1 << 1),  ///< FIFO threshold status on INT_DRDY pin
    INT_F_FULL = (0b1 << 2),  ///< FIFO full flag on INT_DRDY pin
    INT_EN     = (0b1 << 4),  ///< Interrupt signal on INT_DRDY pin
    DRDY       = (0b1 << 5),  ///< Date-ready signal on INT_DRDY pin
    DRDY_PLS   = (0b1 << 6)   ///< Data-ready pulsed on INT_DRDY pin
};

static const uint8_t FIFO_CTRL_addr{0x14};  ///< FIFO control register
enum FIFO_CTRL : uint8_t
{
    BYPASS               = 0b000,
    FIFO                 = 0b001,
    CONTINUOUS           = 0b010,
    BYPASS_TO_FIFO       = 0b101,
    BYPASS_TO_CONTINUOUS = 0b110,
    CONTINUOUS_TO_FIFO   = 0b111,
    STOP_ON_WTM          = (0b1 << 3)  ///< Stop-on-FIFO watermark
};

static const uint8_t FIFO_WTM_addr{0x15};  ///< FIFO threshold setting register
static const uint8_t REF_P_L_addr{0x16};   ///< Reference pressure LSB data
static const uint8_t REF_P_H_addr{0x17};   ///< Reference pressure MSB data
static const uint8_t RPDS_L_addr{0x1a};    ///< Pressure offset LSB data
static const uint8_t RPDS_H_addr{0x1b};    ///< Pressure offset HSB data
static const uint8_t INT_SOURCE_addr{0x24};  ///< Interrupt source register
enum INT_SOURCE : uint8_t
{
    PH      = (0b1 << 0),  ///< Differential pressure High
    PL      = (0b1 << 1),  ///< Differential pressure Low
    IA      = (0b1 << 2),  ///< Interrupt active
    BOOT_ON = (0b1 << 7)   ///< Reboot phase is running
};

static const uint8_t FIFO_STATUS1_addr{0x25};  ///< FIFO status register 1
static const uint8_t FIFO_STATUS2_addr{0x26};  ///< FIFO status register 2
enum FIFO_STATUS2 : uint8_t
{
    FIFO_FULL_IA = (0b1 << 5),  ///< FIFO full status
    FIFO_OVR_IA  = (0b1 << 6),  ///< FIFO overrun status
    FIFO_WTM_IA  = (0b1 << 7)   ///< FIFO threshold status
};

static const uint8_t STATUS_addr{0x27};  ///< Status register
enum STATUS : uint8_t
{
    P_DA = (0b1 << 0),  ///< Pressure data available
    T_DA = (0b1 << 1),  ///< Temperature data available
    P_OR = (0b1 << 4),  ///< Pressure data overrun
    T_OR = (0b1 << 5)   ///< Temperature data overrun
};

static const uint8_t PRESS_OUT_XL_addr{
    0x28};  ///< Pressure output value LSB data
static const uint8_t PRESS_OUT_L_addr{
    0x29};  ///< Pressure output value middle data
static const uint8_t PRESS_OUT_H_addr{
    0x2a};  ///< Pressure output value MSB data
static const uint8_t TEMP_OUT_L_addr{
    0x2b};  ///< Temperature output value LSB data
static const uint8_t TEMP_OUT_H_addr{
    0x2c};  ///< Temperature output value MSB data
static const uint8_t FIFO_DATA_OUT_PRESS_XL_addr{
    0x78};  ///< FIFO pressure output LSB data
static const uint8_t FIFO_DATA_OUT_PRESS_L_addr{
    0x79};  ///< FIFO pressure output middle data
static const uint8_t FIFO_DATA_OUT_PRESS_H_addr{
    0x7a};  ///< FIFO pressure output MSB data
}  // namespace LPS28DFWDefs
}  // namespace Boardcore