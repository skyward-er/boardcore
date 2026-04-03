/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Leonardo Montecchi
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

#define MIOSIX_SENSOR_ND015A_SPI SPI1
#define MIOSIX_SENSOR_ND030D_SPI SPI1


namespace miosix
{

namespace interfaces
{

// ND015A, ND030D
namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1



// Thermocouple
namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4


// Miosix UART
namespace usart1
{
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace usart1

// HIL UART
namespace uart4
{
using tx = Gpio<GPIOA_BASE, 0>;
using rx = Gpio<GPIOA_BASE, 1>;
}  // namespace uart4

namespace can1
{
using tx = Gpio<GPIOA_BASE, 12>;
using rx = Gpio<GPIOA_BASE, 11>;
}  // namespace can1

namespace timers
{
using tim1ch1 = Gpio<GPIOA_BASE, 8>;
using tim3ch1 = Gpio<GPIOC_BASE, 6>;
using tim4ch2 = Gpio<GPIOD_BASE, 13>;
using tim9ch1 = Gpio<GPIOA_BASE, 2>;
}  // namespace timers



}  // namespace interfaces

namespace sensors
{

namespace ND015A
{
using cs = Gpio<GPIOC_BASE, 4>;
}

namespace ND030D
{
using cs = Gpio<GPIOC_BASE, 5>;
}

namespace Thermocouple
{
using cs = Gpio<GPIOC_BASE, 7>;
}

}  // namespace sensors

namespace HeatingPad
{
using measure  = Gpio<GPIOB_BASE, 1>;
using enable     = Gpio<GPIOB_BASE, 11>;
using sense     = Gpio<GPIOB_BASE, 12>;
}  // namespace HeatingPad

namespace gpios
{
using boardLed   = Gpio<GPIOB_BASE, 12>;
using statusLed  = Gpio<GPIOB_BASE, 13>;
}  // namespace gpios

}  // namespace miosix
