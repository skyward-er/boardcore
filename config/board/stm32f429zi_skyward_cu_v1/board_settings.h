/***************************************************************************
 *   Copyright (c) 2019 Skyward Experimental Rocketry                      *
 *   Authors: Silvano Seva, Alberto Nidasio                                *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

#pragma once

#include "interfaces/gpio.h"

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
/// The C standard library is stack-heavy (iprintf requires 1KB) but the
/// STM32F407VG only has 192KB of RAM so there is room for a big 4K stack.
const unsigned int MAIN_STACK_SIZE = 16 * 1024;

/// Clock options
enum class OscillatorType { HSI, HSE };
// Supported oscillator types: HSE
constexpr auto oscillatorType=OscillatorType::HSE;
constexpr unsigned int hseFrequency=25000000;
// Supported clock frequencies: 180000000, 168000000
// Note: at 180Mhz SDIO and RNG run at the wrong speed and USB will not work
// because the PLL will run at 45MHz instead of 48MHz due to hardware
// limitations 
constexpr unsigned int cpuFrequency=168000000;

/// Serial port
const unsigned int defaultSerial      = 1;
const unsigned int defaultSerialSpeed = 115200;
const bool defaultSerialFlowctrl      = false;
// Serial 1 has no DMA as it would conflict with SPI6
const bool defaultSerialDma=true;
// Default serial 1 pins (uncomment when using serial 1)
using defaultSerialTxPin = Gpio<PA,9>;
using defaultSerialRxPin = Gpio<PA,10>;
using defaultSerialRtsPin = Gpio<PA,12>;
using defaultSerialCtsPin = Gpio<PA,11>;

// #define I2C_WITH_DMA

// SD card driver
static const unsigned char sdVoltage = 33;  // Board powered @ 3.3V
#ifdef __ENABLE_XRAM
// Reduce SD clock to ~4.8MHz
#define OVERRIDE_SD_CLOCK_DIVIDER_MAX 8
#endif  //__ENABLE_XRAM
// #define SD_ONE_BIT_DATABUS //This board supports 4 bit databus to SD card

/// Analog supply voltage for ADC, DAC, Reset blocks, RCs and PLL
#define V_DDA_VOLTAGE 3.3f

/**
 * \}
 */

}  // namespace miosix

