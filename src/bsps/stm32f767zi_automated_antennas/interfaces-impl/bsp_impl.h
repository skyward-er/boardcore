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
typedef Gpio<GPIOB_BASE, 7> userLed1;
typedef Gpio<GPIOE_BASE, 3> userLed2;
typedef Gpio<GPIOC_BASE, 13> userLed3_1;
typedef Gpio<GPIOG_BASE, 9> userLed3_2;
typedef Gpio<GPIOC_BASE, 2> userLed4;
typedef Gpio<GPIOB_BASE, 2> userSwitch;

inline void ledOn()
{
    userLed1::high();
    userLed2::high();
    userLed3_1::high();
    userLed3_2::high();
    userLed4::high();
}

inline void ledOff()
{
    userLed1::low();
    userLed2::low();
    userLed3_1::low();
    userLed3_2::low();
    userLed4::low();
}

inline void led1On() { userLed1::high(); }

inline void led1Off() { userLed1::low(); }

inline void led2On() { userLed2::high(); }

inline void led2Off() { userLed2::low(); }

inline void led3On()
{
    userLed3_1::high();
    userLed3_2::high();
}

inline void led3Off()
{
    userLed3_1::low();
    userLed3_2::low();
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
