/* Copyright (c) 2022 Skyward Experimental Rocketry
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
using miso = Gpio<GPIOB_BASE, 4>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

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

// GPS UART
namespace uart2
{
using tx = Gpio<GPIOA_BASE, 2>;
using rx = Gpio<GPIOA_BASE, 3>;
}  // namespace uart2

namespace timers
{
using tim4ch2  = Gpio<GPIOB_BASE, 7>;  // Servo 1
using tim10ch1 = Gpio<GPIOF_BASE, 6>;  // Servo 2
}  // namespace timers

}  // namespace interfaces

namespace sensors
{

namespace mpu9250
{
using cs   = Gpio<GPIOB_BASE, 2>;
using sck  = interfaces::spi1::sck;
using miso = interfaces::spi1::miso;
using mosi = interfaces::spi1::mosi;
}  // namespace mpu9250

namespace bme280
{
using cs   = Gpio<GPIOC_BASE, 11>;
using sck  = interfaces::spi1::sck;
using miso = interfaces::spi1::miso;
using mosi = interfaces::spi1::mosi;
}  // namespace bme280

namespace gps
{
using tx = interfaces::uart2::tx;
using rx = interfaces::uart2::rx;
}  // namespace gps

namespace adc
{
using battery = Gpio<GPIOC_BASE, 3>;
}

}  // namespace sensors

namespace sx1278
{
using cs        = Gpio<GPIOC_BASE, 1>;
using interrupt = Gpio<GPIOF_BASE, 10>;
using sck       = interfaces::spi4::sck;
using miso      = interfaces::spi4::miso;
using mosi      = interfaces::spi4::mosi;
}  // namespace sx1278

namespace servos
{
using servo1 = interfaces::timers::tim4ch2;
using servo2 = interfaces::timers::tim10ch1;
}  // namespace servos

namespace ui
{
using button   = Gpio<GPIOA_BASE, 0>;   // User button
using greenLed = Gpio<GPIOG_BASE, 13>;  // Green LED
using redLed   = Gpio<GPIOG_BASE, 14>;  // Red LED
}  // namespace ui

}  // namespace miosix
