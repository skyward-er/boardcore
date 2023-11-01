/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#ifndef BOARD_SETTINGS_H
#define BOARD_SETTINGS_H

#include "util/version.h"

/**
 * \internal
 * Versioning for board_settings.h for out of git tree projects
 */
#define BOARD_SETTINGS_VERSION 300

namespace miosix
{

/**
 * \addtogroup Settings
 * \{
 */

/// Size of stack for main().
/// The C standard library is stack-heavy (iprintf requires 1.5KB) and the
/// STM32F205RC has 128KB of RAM so there is room for a big 4K stack.
const unsigned int MAIN_STACK_SIZE = 4 * 1024;

/// Serial port
const unsigned int defaultSerial      = 1;
const unsigned int defaultSerialSpeed = 115200;
const bool defaultSerialFlowctrl      = false;
#define SERIAL_1_DMA
// #define SERIAL_2_DMA //Serial 2 is used by the pogo pins
// #define SERIAL_3_DMA  //Serial 3 is used by the pogo pins

// SD card driver
static const unsigned char sdVoltage = 33;  // Board powered @ 3.3V
// #define SD_ONE_BIT_DATABUS // This board supports 4 bit data bus to SD card

/// Analog supply voltage for ADC, DAC, Reset blocks, RCs and PLL
#define V_DDA_VOLTAGE 3.3f

/**
 * \}
 */

}  // namespace miosix

#endif /* BOARD_SETTINGS_H */
