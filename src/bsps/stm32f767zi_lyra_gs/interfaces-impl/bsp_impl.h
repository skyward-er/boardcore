/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Nicolò Caruso
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

/***************************************************************************
 * bsp_impl.h Part of the Miosix Embedded OS.
 * Board support package, this file initializes hardware.
 ***************************************************************************/

#pragma once

#include "config/miosix_settings.h"
#include "hwmapping.h"
#include "interfaces/gpio.h"

namespace miosix
{
inline void ledOn()
{
    userLed1::high();
    userLed2::high();
    userLed3::high();
    userLed4::high();
}

inline void ledOff()
{
    userLed1::low();
    userLed2::low();
    userLed3::low();
    userLed4::low();
}

/**
 * @brief GREEN led on (CU)
 */
inline void led1On() { userLed1::high(); }

inline void led1Off() { userLed1::low(); }

/**
 * @brief YELLOW led on (CU)
 */
inline void led2On() { userLed2::high(); }

inline void led2Off() { userLed2::low(); }

/**
 * @brief RED led on (CU)
 */
inline void led3On() { userLed3::high(); }

inline void led3Off() { userLed3::low(); }

/**
 * @brief ORANGE led on (CU)
 */
inline void led4On() { userLed4::high(); }

inline void led4Off() { userLed4::low(); }

/**
 * \internal
 * Called by stage_1_boot.cpp to enable the SDRAM before initializing .data/.bss
 * Requires the CPU clock to be already configured (running from the PLL)
 */
void configureSdram();

/**
 * Polls the SD card sense GPIO.
 *
 * This board has no SD card whatsoever, but a card can be connected to the
 * following GPIOs:
 * TODO: never tested
 *
 * \return true. As there's no SD card sense switch, let's pretend that
 * the card is present.
 */
inline bool sdCardSense() { return true; }

}  // namespace miosix