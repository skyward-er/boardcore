/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Vincenzo Tirolese
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

#ifndef HWMAPPING_H
#define HWMAPPING_H
#endif

#include "interfaces/gpio.h"

namespace miosix
{

namespace interfaces
{

// spi LSM6DSRX
namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

// spi gps,adc
namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

// USB UART
namespace uart1
{
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace uart1

// HIL UART
namespace uart2
{
using tx = Gpio<GPIOA_BASE, 2>;
using rx = Gpio<GPIOA_BASE, 3>;
}  // namespace uart2

// SIM900 UART
namespace uart4
{
using tx = Gpio<GPIOA_BASE, 0>;
using rx = Gpio<GPIOA_BASE, 1>;
}  // namespace uart4

}  // namespace interfaces

namespace sensors
{

namespace LSM6DSRX
{
using cs = Gpio<GPIOC_BASE, 4>;

}  // namespace LSM6DSRX

namespace ads131m04
{
using cs = Gpio<GPIOE_BASE, 4>;
}  // namespace ads131m04

namespace ubxgps
{
using cs = Gpio<GPIOE_BASE, 3>;
}  // namespace ubxgps

}  // namespace sensors

namespace leds
{
using led_red1  = Gpio<GPIOB_BASE, 0>;
using led_green = Gpio<GPIOB_BASE, 1>;
using led_ext   = Gpio<GPIOB_BASE, 10>;

/**
 * These are connected to the enable pin of the thermal cutters and the cs of
 * the lis3mdl magnetometer.
 */
// using led_blue2  = Gpio<GPIOG_BASE, 2>;
// using led_green1 = Gpio<GPIOG_BASE, 6>;
// using led_green2 = Gpio<GPIOD_BASE, 11>;
}  // namespace leds
}  // namespace miosix
