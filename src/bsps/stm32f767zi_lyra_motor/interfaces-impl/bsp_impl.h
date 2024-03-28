/* Copyright (c) 2023 Skyward Experimental Rocketry
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

/***************************************************************************
 * bsp_impl.h Part of the Miosix Embedded OS.
 * Board support package, this file initializes hardware.
 ***************************************************************************/

#ifndef BSP_IMPL_H
#define BSP_IMPL_H

#include "config/miosix_settings.h"
#include "interfaces/gpio.h"

namespace miosix
{

/**
\addtogroup Hardware
\{
*/

/**
 * \internal
 * Called by stage_1_boot.cpp to enable the SDRAM before initializing .data/.bss
 * Requires the CPU clock to be already configured (running from the PLL)
 */
void configureSdram();

/**
 * \internal
 * Board pin definition
 */
typedef Gpio<GPIOC_BASE, 15> userLed1;
typedef Gpio<GPIOC_BASE, 2> userLed2;
typedef Gpio<GPIOC_BASE, 13> userLed3;
typedef Gpio<GPIOC_BASE, 14> userLed4;

inline void led1On() { userLed1::high(); }

inline void led1Off() { userLed1::low(); }

inline void led2On() { userLed2::high(); }

inline void led2Off() { userLed2::low(); }

inline void led3On() { userLed3::high(); }

inline void led3Off() { userLed3::low(); }

inline void led4On() { userLed4::high(); }

inline void led4Off() { userLed4::low(); }

inline void ledOn()
{
    led1On();
    led2On();
    led3On();
    led4On();
}

inline void ledOff()
{
    led1Off();
    led2Off();
    led3Off();
    led4Off();
}

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

/**
\}
*/

}  // namespace miosix

#endif  // BSP_IMPL_H
