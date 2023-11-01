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

/***********************************************************************
 * bsp_impl.h Part of the Miosix Embedded OS.
 * Board support package, this file initializes hardware.
 ************************************************************************/

#ifndef BSP_IMPL_H
#define BSP_IMPL_H

#include "config/miosix_settings.h"
#include "drivers/stm32_hardware_rng.h"
#include "hwmapping.h"
#include "interfaces/gpio.h"

namespace miosix
{

/**
\addtogroup Hardware
\{
*/

inline void ledOn()
{
    devices::leds::led1::high();
    devices::leds::led2::high();
    devices::leds::led3::high();
    devices::leds::led4::high();
}

inline void ledOff()
{
    devices::leds::led1::low();
    devices::leds::led2::low();
    devices::leds::led3::low();
    devices::leds::led4::low();
}

/**
 * Polls the SD card sense GPIO
 * \return true if there is an uSD card in the socket.
 */
inline bool sdCardSense() { return true; }

/**
\}
*/

}  // namespace miosix

#endif  // BSP_IMPL_H
