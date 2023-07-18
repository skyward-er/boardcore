/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

namespace ADS131M04Defs
{

static constexpr int CHANNELS_NUM            = 4;
static constexpr int CALIBRATION_SAMPLES     = 250;
static constexpr int SELF_TEST_SAMPLES       = 250;
static constexpr int FULL_FRAME_SIZE         = 18;
static constexpr uint16_t RESET_CMD_RESPONSE = 0xFF24;
static constexpr uint16_t WRITE_CMD_RESPONSE = 0x4000;

///< Digit value in mV for each pga configurations
constexpr float PGA_LSB_SIZE[8] = {143.0511e-9, 71.5256e-9, 35.7628e-9,
                                   17.8814e-9,  8.9407e-9,  4.4703e-9,
                                   2.2352e-9,   1.1176e-9};

static constexpr float V_REF              = 1.2;
static constexpr float TEST_SIGNAL_FACTOR = 2.0 / 15.0;
static constexpr float TEST_SIGNAL_SLACK  = 0.1;  // Not defined in DS

/**
 * @brief ADC's oversampling ratio configurations.
 *
 * The OSR determins the output data rate, depending on the master clock
 * frequency.
 *
 * ODR = f_CLK / 2 / OSR
 *
 * On Skyward's boards an 8.192MHz clock is used.
 */
enum class OversamplingRatio : uint16_t
{
    OSR_128   = 0,         // ODR is 32KHz
    OSR_256   = 0x1 << 2,  // ODR is 16KHz
    OSR_512   = 0x2 << 2,  // ODR is 8KHz
    OSR_1024  = 0x3 << 2,  // ODR is 4KHz
    OSR_2048  = 0x4 << 2,  // ODR is 2KHz
    OSR_4096  = 0x5 << 2,  // ODR is 1KHz
    OSR_8192  = 0x6 << 2,  // ODR is 500Hz
    OSR_16256 = 0x7 << 2   // ODR is 250Hz
};

enum class PGA : uint16_t
{
    PGA_1   = 0,    ///< Full scale resolution is ±1.2V
    PGA_2   = 0x1,  ///< Full scale resolution is ±600mV
    PGA_4   = 0x2,  ///< Full scale resolution is ±300mV
    PGA_8   = 0x3,  ///< Full scale resolution is ±150mV
    PGA_16  = 0x4,  ///< Full scale resolution is ±75mV
    PGA_32  = 0x5,  ///< Full scale resolution is ±37.5mV
    PGA_64  = 0x6,  ///< Full scale resolution is ±18.75mV
    PGA_128 = 0x7   ///< Full scale resolution is ±9.375mV
};

enum class Channel : uint8_t
{
    CHANNEL_0 = 0,
    CHANNEL_1 = 1,
    CHANNEL_2 = 2,
    CHANNEL_3 = 3
};

enum class Input : uint8_t
{
    DEFAULT          = 0,  // AINxP and AINxN (default)
    SHORTED          = 1,  // ADC inputs shorted
    POSITIVE_DC_TEST = 2,  // Positive DC test signal
    NEGATIVE_DC_TEST = 3   // Negative DC test signal
};

enum class Register : uint16_t
{
    // Device settings and indicators
    REG_ID     = 0,
    REG_STATUS = 0x1,

    // Global settings across channels
    REG_MODE        = 0x2,
    REG_CLOCK       = 0x3,
    REG_GAIN        = 0x4,
    REG_CFG         = 0x6,
    REG_THRSHLD_MSB = 0x7,
    REG_THRSHLD_LSB = 0x8,

    // Channel specific settings
    REG_CH0_CFG      = 0x9,
    REG_CH0_OCAL_MSB = 0xA,
    REG_CH0_OCAL_LSB = 0xB,
    REG_CH0_GCAL_MSB = 0xC,
    REG_CH0_GCAL_LSB = 0xD,
    REG_CH1_CFG      = 0xE,
    REG_CH1_OCAL_MSB = 0xF,
    REG_CH1_OCAL_LSB = 0x10,
    REG_CH1_GCAL_MSB = 0x11,
    REG_CH1_GCAL_LSB = 0x12,
    REG_CH2_CFG      = 0x13,
    REG_CH2_OCAL_MSB = 0x14,
    REG_CH2_OCAL_LSB = 0x15,
    REG_CH2_GCAL_MSB = 0x16,
    REG_CH2_GCAL_LSB = 0x17,
    REG_CH3_CFG      = 0x18,
    REG_CH3_OCAL_MSB = 0x19,
    REG_CH3_OCAL_LSB = 0x1A,
    REG_CH3_GCAL_MSB = 0x1B,
    REG_CH3_GCAL_LSB = 0x1C,

    // Register map CRC
    REG_REGMAP_CRC = 0x3E
};

enum class Command : uint16_t
{
    NULL_CMD = 0x0000,
    RESET    = 0x0011,
    STANDBY  = 0x0022,
    WAKEUP   = 0x0033,
    LOCK     = 0x0555,
    UNLOCK   = 0x0655,
    RREG     = 0xA000,
    WREG     = 0x6000
};

namespace RegStatusMasks
{
constexpr uint16_t LOCK     = 0x1 << 15;
constexpr uint16_t F_RESYNC = 0x1 << 14;
constexpr uint16_t REG_MAP  = 0x1 << 13;
constexpr uint16_t CRC_ERR  = 0x1 << 12;
constexpr uint16_t CRC_TYPE = 0x1 << 11;
constexpr uint16_t RESET    = 0x1 << 10;
constexpr uint16_t WLENGTH  = 0x3 << 8;
constexpr uint16_t DRDY3    = 0x1 << 3;
constexpr uint16_t DRDY2    = 0x1 << 2;
constexpr uint16_t DRDY1    = 0x1 << 1;
constexpr uint16_t DRDY0    = 0x1 << 0;
}  // namespace RegStatusMasks

namespace RegModeMasks
{
constexpr uint16_t REG_CRC_EN = 0x1 << 13;
constexpr uint16_t RX_CRC_EN  = 0x1 << 12;
constexpr uint16_t CRC_TYPE   = 0x1 << 11;
constexpr uint16_t RESET      = 0x1 << 10;
constexpr uint16_t WLENGTH    = 0x3 << 8;
constexpr uint16_t TIMEOUT    = 0x1 << 4;
constexpr uint16_t DRDY_SEL   = 0x3 << 2;
constexpr uint16_t DRDY_HiZ   = 0x1 << 1;
constexpr uint16_t DRDY_FMT   = 0x1 << 0;
}  // namespace RegModeMasks

namespace RegClockMasks
{
constexpr uint16_t CH3_EN     = 0x1 << 11;
constexpr uint16_t CH2_EN     = 0x1 << 10;
constexpr uint16_t CH1_EN     = 0x1 << 9;
constexpr uint16_t CH0_EN     = 0x1 << 8;
constexpr uint16_t OSR        = 0x7 << 2;
constexpr uint16_t POWER_MODE = 0x3 << 0;
}  // namespace RegClockMasks

namespace RegGainMasks
{
constexpr uint16_t PGA_GAIN_3 = 0x7 << 12;
constexpr uint16_t PGA_GAIN_2 = 0x7 << 8;
constexpr uint16_t PGA_GAIN_1 = 0x7 << 4;
constexpr uint16_t PGA_GAIN_0 = 0x7 << 0;
}  // namespace RegGainMasks

namespace RegConfigurationMasks
{
constexpr uint16_t GC_DLY   = 0xF << 9;
constexpr uint16_t GC_EN    = 0x1 << 8;
constexpr uint16_t CD_ALLCH = 0x1 << 7;
constexpr uint16_t CD_NUM   = 0x7 << 4;
constexpr uint16_t CD_LEN   = 0x7 << 1;
constexpr uint16_t CD_EN    = 0x1 << 0;
}  // namespace RegConfigurationMasks

namespace RegChannelMasks
{
constexpr uint16_t CFG_PHASE     = 0x3FF << 6;
constexpr uint16_t CFG_DCBLK_DIS = 0x001 << 2;
constexpr uint16_t CFG_MUX       = 0x003 << 0;
}  // namespace RegChannelMasks

}  // namespace ADS131M04Defs

}  // namespace Boardcore
