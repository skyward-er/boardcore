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

#include "interfaces/gpio.h"

namespace miosix
{

namespace interfaces
{

namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

namespace spi3
{
using sck  = Gpio<GPIOB_BASE, 3>;
using miso = Gpio<GPIOB_BASE, 4>;
using mosi = Gpio<GPIOD_BASE, 6>;
}  // namespace spi3

namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

namespace usart1
{
using tx = Gpio<GPIOA_BASE, 2>;
using rx = Gpio<GPIOA_BASE, 3>;
}  // namespace usart1

namespace usart2
{
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace usart2

namespace can2
{
using tx = Gpio<GPIOB_BASE, 12>;
using rx = Gpio<GPIOB_BASE, 13>;
}  // namespace can2

}  // namespace interfaces

namespace peripherals
{

namespace leds
{
using userLed1   = Gpio<GPIOB_BASE, 7>;
using userLed2   = Gpio<GPIOE_BASE, 3>;
using userLed3_1 = Gpio<GPIOC_BASE, 13>;  // On MCU rev 2
using userLed3_2 = Gpio<GPIOG_BASE, 9>;   // On MCU rev 3
using userLed4   = Gpio<GPIOC_BASE, 2>;
}  // namespace leds

namespace switches
{
using userSwitch1 = Gpio<GPIOB_BASE, 2>;
}

namespace lsm6dsrx
{
using cs   = Gpio<GPIOC_BASE, 4>;
using int1 = Gpio<GPIOD_BASE, 13>;
using int2 = Gpio<GPIOG_BASE, 7>;
}  // namespace lsm6dsrx

namespace h3lis331dl
{
using cs   = Gpio<GPIOD_BASE, 3>;
using int1 = Gpio<GPIOC_BASE, 3>;
}  // namespace h3lis331dl

namespace lis2mdl
{
using cs = Gpio<GPIOD_BASE, 5>;
}  // namespace lis2mdl

namespace lps22df
{
using cs   = Gpio<GPIOD_BASE, 7>;
using int1 = Gpio<GPIOB_BASE, 11>;
}  // namespace lps22df

namespace ads131m08
{
using cs = Gpio<GPIOG_BASE, 10>;
}  // namespace ads131m08

namespace max31856
{
using cs = Gpio<GPIOD_BASE, 4>;
}  // namespace max31856

namespace servos
{
using servo1 = Gpio<GPIOC_BASE, 6>;  // TIM8 CH1
using servo2 = Gpio<GPIOC_BASE, 7>;  // TIM8 CH2
}  // namespace servos

namespace tank_level
{
using lvl1 = Gpio<GPIOB_BASE, 1>;
using lvl2 = Gpio<GPIOG_BASE, 6>;
using lvl3 = Gpio<GPIOB_BASE, 14>;
}  // namespace tank_level

namespace battery_voltage
{
using ch15 = Gpio<GPIOC_BASE, 5>;
}

}  // namespace peripherals

}  // namespace miosix
