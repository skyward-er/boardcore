/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Riccardo Sironi
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

#include <cstdint>

namespace Boardcore
{
namespace PCA9685Defs
{
enum class Error : uint8_t
{
    NO_ERROR = 0,
    ALREADY_INIT,
    BUS_FAULT,  // The I2C bus failed
    INVALID_ARGS
};

enum class OutputType : uint8_t
{
    OPEN_DRAIN,
    TOTEM_POLE
};

enum class Channel : uint8_t
{
    CHANNEL_0  = 0,
    CHANNEL_1  = 1,
    CHANNEL_2  = 2,
    CHANNEL_3  = 3,
    CHANNEL_4  = 4,
    CHANNEL_5  = 5,
    CHANNEL_6  = 6,
    CHANNEL_7  = 7,
    CHANNEL_8  = 8,
    CHANNEL_9  = 9,
    CHANNEL_10 = 10,
    CHANNEL_11 = 11,
    CHANNEL_12 = 12,
    CHANNEL_13 = 13,
    CHANNEL_14 = 14,
    CHANNEL_15 = 15
};

enum Register : uint8_t
{
    MODE1         = 0x00,
    MODE2         = 0x01,
    PRE_SCALE     = 0xFE,
    ALLCALLADR    = 0xE0,
    CHANNEL_BASE  = 0x06,  // Base address for channel 0
    ALL_LED_ON_L  = 0xFA,
    ALL_LED_ON_H  = 0xFB,
    ALL_LED_OFF_L = 0xFC,
    ALL_LED_OFF_H = 0xFD
};

enum Mode1BitMask : uint8_t
{
    RESTART = 0x80,  // Not used in this driver
    EXTCLK  = 0x40,
    AI      = 0x20,
    SLEEP   = 0x10,
    ALLCALL = 0x01
};

enum Mode2BitMask : uint8_t
{
    TOTEM_POLE = 0x04,
    INVERT     = 0x10
};
}  // namespace PCA9685Defs
}  // namespace Boardcore
